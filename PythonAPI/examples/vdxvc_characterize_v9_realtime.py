#!/usr/bin/env python3
"""
vdxvc_characterize.py
VDxVC Vehicle Dynamics Characterization Suite
=============================================
Implements ISO 4138 (Method 1) · ISO 7401 (step steer) · Throttle Step · ISO 3888-2 proxy

Usage
-----
    python vdxvc_characterize.py --list-vehicles
    python vdxvc_characterize.py --vehicle vehicle.nissan.micra
    python vdxvc_characterize.py --vehicle vehicle.mercedes.slc --test iso7401
    python vdxvc_characterize.py --vehicle vehicle.tesla.model3 --test all --output ./results
    python vdxvc_characterize.py --vehicle vehicle.nissan.micra --circle-control feedforward

    --vehicle accepts any blueprint ID returned by --list-vehicles.

Environment
-----------
Run in synchronous mode. This client owns world.tick().
Do NOT run alongside an IVR client that also ticks.
Start CARLA server standalone first:
    CarlaUE4.exe -carla-server -windowed -ResX=640

Coordinate conventions (CARLA body frame, flat ground)
------------------------------------------------------
    X  : forward   (+)
    Y  : left       (+)   CARLA convention; ISO 8855 intermediate frame is X-forward Y-left
    Z  : up         (+)
IMU accelerometer.y = lateral acceleration  (positive = leftward = centripetal in left turn)
IMU accelerometer.z = vertical + gravity    (~9.81 m/s^2 upward at rest) — logged as "gravity"
IMU gyroscope.z     = yaw rate psi_dot      (positive = turning left / CCW from above)

ISO 8855 sign note for analysts
--------------------------------
ISO 8855 intermediate frame: lateral acceleration positive to the LEFT of travel direction.
CARLA IMU ay (+left) matches ISO 8855 on flat ground with no roll correction needed.
To convert to SAE J670e (ay positive = right): ay_SAE = -ay in post-processing.

Authors : VDxVC Research Team
Version : 9.0 | June 2026
Changes from the previous calibration round (the ISO 7401 units fix):
    - ISO 7401 calibration: fixed a units mismatch in the Ay-error term. The
      prior round compared the g-valued target against the m/s^2 IMU reading, so
      the loop converged to ay = target/9.81 (about an order of magnitude low,
      with ay_target_reached=false at every level). The measured ay is now
      converted to g before the difference. The closed-loop design itself was
      sound — left and right converged to identical plateaus — only the error
      units were wrong.
Changes from v3.0 (calibration-round fixes — informed by the mercedes_vr batches):
    - ISO 4138 circle PI now acts on RELATIVE radius error (R_meas-R)/R, not raw
      metres. One gain set (CIRCLE_KP=1.5, CIRCLE_KI=0.6) now holds the target at
      any radius; v3 left a ~20 % steady-state overshoot (60 m at a 50 m target).
    - ISO 4138 understeer-gradient K is fitted only in the linear window
      [AY_FIT_MIN_G, AY_FIT_MAX_G] and rejects plateaus whose radius drifted
      beyond R_TOLERANCE_FRAC of target. This removes the non-monotonic
      understeer->oversteer transition and handling-limit plateaus that made
      K_left and K_right disagree by 70 %. Summary now reports per-direction R²,
      the fit window, point counts, and a K_left_right_asymmetric flag.
    - ISO 7401 §10.1 amplitude calibration is now CLOSED-LOOP on lateral
      acceleration: a PI steers until the measured Ay reaches the target, then
      the held SWA is read. v3 held an open-loop kinematic radius, which at high
      speed needed sub-degree steer the loop could not hold, so the measured Ay
      came out ~100x below target. Each calibration now carries an
      ay_target_reached flag.
    - Throttle step: a 0.4 s coast precedes the full-throttle step so every run
      starts from the same powertrain state; peak detection skips the step-edge
      settling samples and jerk is computed on a short moving average. This fixes
      the t_peak=0 artifact at 80 km/h and the spurious identical jerk at 60/80.
Changes from v2.1:
    - ISO 4138: radius control mode selectable via --circle-control {pi|feedforward}
    - ISO 4138 and ISO 7401: both directions (left + right) + 3 repetitions each
    - az renamed to gravity throughout (IMU vertical channel)
    - ay_ss sanity clamp: warns if |ay_ss| > 10 m/s^2 (impossible on flat ground)
    - ISO 7401 steer amplitude levels now ISO-compliant: 2, 4, 6 m/s^2 target Ay
    - Greek-letter variable names replaced with ASCII (psi_dot, delta, beta, ...)
    - delta_dot logged as steering rate proxy (d(delta_H)/dt)
"""

import argparse
import json
import math
import queue
import random
import sys
import time
from datetime import datetime
from pathlib import Path

import carla
import numpy as np
import pandas as pd


# ==============================================================================
# VEHICLE SELECTION
# --vehicle accepts any blueprint ID available on the connected CARLA server.
# Use --list-vehicles to print all valid IDs before running a test.
# Examples:
#   python vdxvc_characterize.py --vehicle vehicle.nissan.micra
#   python vdxvc_characterize.py --vehicle vehicle.mercedes.slc --test iso7401
#   python vdxvc_characterize.py --list-vehicles
# The blueprint ID is passed directly; no registry or alias lookup is performed.
# ==============================================================================


# ==============================================================================
# GLOBAL PHYSICS CONSTANTS
# ==============================================================================
PHYSICS_DT      = 0.01          # 100 Hz — ISO minimum for transient measurements
GRAVITY_MS2     = 9.81          # m/s^2

# -- ISO 4138 thresholds -------------------------------------------------------
AY_LIMIT_G              = 0.6   # Upper Ay boundary for linear-region K regression
AY_PLATEAU_THRESHOLD_G  = 0.005 # Convergence: stop ramp if Ay change < this over 3 steps
AY_INCREMENT_WARN_G     = 0.05  # ISO 4138 §8.3: step <= 0.5 m/s^2 ~ 0.051 g

# K (understeer-gradient) regression window. v8 change: the SWA-vs-Ay curve is
# often non-monotonic (mild understeer near the limit of adhesion gives way to
# oversteer), so fitting across the whole 0-0.6 g range mixes two regimes and
# makes K_left and K_right disagree. We fit only the low-Ay LINEAR region.
AY_FIT_MIN_G            = 0.10  # lower edge of the linear-region K fit
AY_FIT_MAX_G            = 0.40  # upper edge of the linear-region K fit
# Drop any plateau whose measured radius drifted more than this fraction from
# the target (controller lost the circle / handling limit reached) — those
# points are not steady-state and corrupt the gradient.
R_TOLERANCE_FRAC        = 0.15  # 15 % radius tolerance for a valid plateau
# Warn if the two directions disagree by more than this (alignment/asymmetry).
K_ASYMMETRY_WARN_FRAC   = 0.50  # 50 % L/R spread

# -- ISO 7401 target lateral acceleration levels (§10.1) ----------------------
# Standard: 4 m/s^2. Additional: 2 and 6 m/s^2.
ISO7401_AY_TARGETS_MS2  = [2.0, 4.0, 6.0]
ISO7401_N_REPS          = 3     # repetitions per direction per amplitude

# -- ISO 7401 §10.1 amplitude-calibration controller --------------------------
# v8 change: the step amplitude is now found by a CLOSED-LOOP controller that
# steers until the MEASURED lateral acceleration equals the target, then reads
# the held SWA. v7 opened the loop at a kinematic radius R = v^2/Ay; at 100 km/h
# a 2 m/s^2 target is only ~0.4 deg of steer — too small for the open-loop
# circle to hold — so the vehicle ran nearly straight and the measured Ay came
# out ~100x below target. Controlling on Ay directly guarantees the target is
# actually reached and the logged SWA is physically meaningful.
AY_CAL_KP        = 0.5     # P gain on Ay error [g] -> normalised steer
AY_CAL_KI        = 0.8     # I gain (drives steer to whatever the target needs)
AY_CAL_INT_MAX   = 1.0     # anti-windup clamp on the integral accumulator
AY_CAL_TOL_G     = 0.03    # plateau accepted when |Ay - target| < this (g)

# -- PI speed controller -------------------------------------------------------
# Tune these if speed tracking is poor for a specific vehicle/map.
PI_KP           = 0.5           # Proportional gain
PI_KI           = 0.1           # Integral gain
PI_INTEGRAL_MAX = 1.0           # Anti-windup clamp (normalised throttle units)

# -- ISO 4138 circle-holding PI (RELATIVE radius-error mode) -------------------
# Controls steering to hold actual radius R_meas = v / psi_dot at target R.
#
# v8 change: the error is now the RELATIVE radius error, dimensionless:
#       R_error = direction_sign * (R_meas - R_target) / R_target
# In v7 the error was raw metres, so the same gains behaved completely
# differently at R=50 m vs R=386 m (ISO 7401 calibration radii), and the tiny
# integral clamp left a ~20 % steady-state radius offset (measured 60 m at a
# 50 m target). Normalising by R_target makes one gain set valid at every
# radius, and a 20 % overshoot now reads as error = 0.20 — large enough for the
# integral to act on.
#
# Error convention (unchanged):
#   R_meas > R_target (wandering out) -> positive error -> MORE steer (tighten)
#   R_meas < R_target (cutting in)    -> negative error -> LESS steer (open up)
#
# Tuning:
#   - Increase CIRCLE_KP if R_meas converges too slowly; decrease if it oscillates
#   - CIRCLE_KI removes the steady-state offset; raise if a residual bias remains
#   - CIRCLE_INTEGRAL_MAX bounds the integral accumulator (anti-windup) ONLY;
#     the controller output is clamped to the full steer range [-1, 1] inside
#     circle_steer() so a large transient can still use full steering authority.
CIRCLE_KP            = 1.5    # P gain on RELATIVE radius error -> normalised steer
CIRCLE_KI            = 0.6    # I gain (closes the steady-state radius offset)
CIRCLE_INTEGRAL_MAX  = 0.5    # Anti-windup clamp on the integral accumulator only


# ==============================================================================
# CLI
# ==============================================================================
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="VDxVC ISO Vehicle Dynamics Characterization Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--host",    default="localhost",
                   help="CARLA server hostname (default: localhost)")
    p.add_argument("--port",    default=2000, type=int,
                   help="CARLA server port (default: 2000)")
    p.add_argument("--vehicle", default=None,
                   help="Blueprint ID of the vehicle to characterize "
                        "(e.g. vehicle.nissan.micra). "
                        "Run --list-vehicles to see all IDs available on the server.")
    p.add_argument("--test",    default="all",
                   choices=["all", "iso4138", "iso7401", "throttle", "iso3888"],
                   help="Test to run (default: all)")
    p.add_argument("--output",  default="./vdxvc_characterization",
                   help="Output directory for CSV + JSON (default: ./vdxvc_characterization)")
    p.add_argument("--radius",  default=50.0, type=float,
                   help="ISO 4138 circle radius in metres (default: 50, min: 40)")
    p.add_argument("--map",     default=None,
                   help="CARLA map to load. Omit to keep current map. "
                        "Recommended: flat open map with >150 m clear radius.")
    p.add_argument("--no-render", dest="render", action="store_false",
                   help="Disable rendering (rendering ON by default)")
    p.add_argument("--circle-control", default="pi",
                   choices=["pi", "feedforward"],
                   help="ISO 4138 circle-holding strategy. "
                        "'pi'          : PI on radius error R_meas - R_target; "
                        "               corrects the radius overshoot seen with yaw-rate PI. "
                        "'feedforward' : pure Ackermann steer only, no correction. "
                        "               (default: pi)")
    p.add_argument("--list-vehicles", action="store_true",
                   help="Print all vehicle blueprints available on server and exit")
    p.add_argument("--realtime", action="store_true",
                   help="Pace world.tick() to wall-clock real time (100 Hz). "
                        "REQUIRED when a physical motion platform (e.g. Atomic A3) "
                        "is driven by the simulation while logging seat motion; "
                        "otherwise the sync loop runs faster than real time and "
                        "the recorded seat dynamics are time-distorted.")
    p.set_defaults(render=True)
    return p.parse_args()


# ==============================================================================
# PI CONTROLLER
# Reusable stateful PI with anti-windup clamping.
# Call .reset() between test segments.
# ==============================================================================
class PIController:
    """
    Discrete PI controller with anti-windup integral clamping.

    Parameters
    ----------
    kp          : proportional gain
    ki          : integral gain
    dt          : time step [s]
    output_min  : lower clamp on output
    output_max  : upper clamp on output
    integral_max: symmetric clamp on integral accumulator (anti-windup)
    """
    def __init__(self, kp: float, ki: float, dt: float,
                 output_min: float = 0.0, output_max: float = 1.0,
                 integral_max: float = 1.0):
        self.kp           = kp
        self.ki           = ki
        self.dt           = dt
        self.output_min   = output_min
        self.output_max   = output_max
        self.integral_max = integral_max
        self._integral    = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def step(self, error: float) -> float:
        self._integral = float(np.clip(
            self._integral + error * self.dt,
            -self.integral_max, self.integral_max
        ))
        output = self.kp * error + self.ki * self._integral
        return float(np.clip(output, self.output_min, self.output_max))


# ==============================================================================
# SESSION CONTEXT MANAGER
# Handles: synchronous mode, sensor queue, actor registry, teardown.
# ==============================================================================
class SimSession:
    """
    Context manager for a clean CARLA characterization session.
    Owns world.tick() — do not run alongside an IVR client.
    """

    def __init__(self, client: carla.Client, map_name: str = None,
                 render: bool = True, realtime: bool = False):
        self.client  = client
        self.world   = client.get_world()
        self.render  = render
        self.realtime = realtime          # pace ticks to wall clock (motion platform in the loop)
        self._next_tick_t = None          # wall-clock deadline for the next tick
        self._actors: list              = []
        self._sensor_queue: queue.Queue = queue.Queue()
        self._original_settings         = self.world.get_settings()

        if map_name:
            current = self.world.get_map().name.split("/")[-1]
            if current != map_name:
                print(f"[sim] Loading map: {map_name}  (current: {current})")
                self.world = client.load_world(map_name)
                time.sleep(3.0)

    def __enter__(self) -> "SimSession":
        s = self.world.get_settings()
        s.synchronous_mode    = True
        s.fixed_delta_seconds = PHYSICS_DT
        s.no_rendering_mode   = not self.render
        self.world.apply_settings(s)
        print(f"[sim] Synchronous {PHYSICS_DT * 1000:.0f} ms | "
              f"rendering={'on' if self.render else 'off'}")
        return self

    def __exit__(self, *_):
        print("[sim] Cleaning up actors ...")
        for a in reversed(self._actors):
            if a.is_alive:
                a.destroy()
        self.world.apply_settings(self._original_settings)
        print("[sim] Session closed.")

    # -- actor helpers ---------------------------------------------------------

    def spawn_vehicle(self, blueprint_id: str) -> carla.Vehicle:
        """
        Spawn vehicle at a randomly chosen spawn point.
        Spawn point is logged in every summary JSON for reproducibility.
        On a flat open map all spawn points are equivalent; on town maps
        the random selection avoids repeated use of the same corner/slope.
        """
        bp = self.world.get_blueprint_library().find(blueprint_id)
        if bp is None:
            raise ValueError(
                f"Blueprint '{blueprint_id}' not found. "
                f"Run --list-vehicles to see available IDs."
            )
        sps = self.world.get_map().get_spawn_points()
        if not sps:
            raise RuntimeError("No spawn points found on current map.")
        sp = random.choice(sps)
        vehicle = self.world.spawn_actor(bp, sp)
        self._actors.append(vehicle)
        # Store for SDLP normalization and JSON logging
        self.spawn_transform = sp
        print(f"[sim] Spawned {blueprint_id}  "
              f"x={sp.location.x:.1f}  y={sp.location.y:.1f}  "
              f"yaw={sp.rotation.yaw:.1f} deg")
        return vehicle

    def attach_imu(self, vehicle: carla.Vehicle) -> carla.Sensor:
        """
        Attach zero-noise IMU at vehicle centre of gravity (approx).
        Noise is zeroed for characterization; add realistic noise profiles
        in post-processing if needed for simulator-fidelity analysis.
        """
        bp = self.world.get_blueprint_library().find("sensor.other.imu")
        bp.set_attribute("sensor_tick", str(PHYSICS_DT))
        for attr in [
            "noise_accel_stddev_x", "noise_accel_stddev_y", "noise_accel_stddev_z",
            "noise_gyro_stddev_x",  "noise_gyro_stddev_y",  "noise_gyro_stddev_z",
        ]:
            bp.set_attribute(attr, "0.0")
        imu = self.world.spawn_actor(
            bp,
            carla.Transform(carla.Location(x=0.0, z=0.5)),
            attach_to=vehicle,
        )
        self._actors.append(imu)
        imu.listen(lambda d: self._sensor_queue.put(d))
        return imu

    # -- tick + sensor read ----------------------------------------------------

    def tick(self) -> "carla.IMUMeasurement | None":
        """Advance simulation one step and return the IMU frame.

        With realtime=True, each tick is paced so that one PHYSICS_DT of
        simulated time takes one PHYSICS_DT of wall-clock time (drift-free
        deadline scheduling). If the server cannot keep up (tick slower than
        PHYSICS_DT), a warning is printed once and the deadline is resynced.
        """
        if self.realtime:
            now = time.perf_counter()
            if self._next_tick_t is None:
                self._next_tick_t = now
            sleep_s = self._next_tick_t - now
            if sleep_s > 0:
                time.sleep(sleep_s)
            elif sleep_s < -0.5:           # fell badly behind: resync, warn once
                if not getattr(self, "_rt_warned", False):
                    print("[warn] --realtime: server slower than real time "
                          f"({-sleep_s:.2f} s behind). Seat-motion timing may "
                          "be distorted. Reduce load (e.g. --no-render) or "
                          "accept the drift.")
                    self._rt_warned = True
                self._next_tick_t = time.perf_counter()
            self._next_tick_t += PHYSICS_DT
        self.world.tick()
        try:
            return self._sensor_queue.get(timeout=1.0)
        except queue.Empty:
            print("[warn] IMU queue timeout — check sensor_tick matches PHYSICS_DT")
            return None

    def get_spawn_points(self) -> list:
        return self.world.get_map().get_spawn_points()


# ==============================================================================
# VEHICLE PARAMETER EXTRACTION
# ==============================================================================
def _wheel_xyz(wheel) -> tuple:
    """
    Return (x, y, z) position of a WheelPhysicsControl in cm, vehicle-local frame.

    CARLA UE4 exposes wheel.position as a carla.Vector3D (x, y, z).
    CARLA UE5 removed the Vector3D and may expose position_x / position_y / position_z
    as flat floats, or may not expose wheel positions at all.

    Fallback chain:
        1. wheel.position.x / .y / .z          (UE4 API)
        2. wheel.position_x / _y / _z          (possible UE5 flat API)
        3. (0, 0, 0) with a warning            (API unknown — params estimated later)

    All values are in centimetres, vehicle-local frame (X forward, Y left, Z up).
    """
    # attempt 1: UE4-style Vector3D
    pos = getattr(wheel, "position", None)
    if pos is not None and hasattr(pos, "x"):
        return float(pos.x), float(pos.y), float(pos.z)

    # attempt 2: UE5-style flat floats
    px = getattr(wheel, "position_x", None)
    py = getattr(wheel, "position_y", None)
    pz = getattr(wheel, "position_z", None)
    if px is not None:
        return float(px), float(py or 0.0), float(pz or 0.0)

    # fallback
    print("[warn] WheelPhysicsControl has no position attribute — "
          "wheelbase and track will be estimated from bounding box.")
    return 0.0, 0.0, 0.0


def get_vehicle_params(vehicle: carla.Vehicle) -> dict:
    """
    Extract geometric parameters from VehiclePhysicsControl.

    Wheel position API differs between CARLA UE4 and UE5 (see _wheel_xyz).
    If wheel positions are unavailable, wheelbase and track are estimated from
    the vehicle bounding box (less accurate but sufficient for PI feed-forward).

    Wheel index convention: [0]=FL  [1]=FR  [2]=RL  [3]=RR
    All wheel positions in cm, vehicle-local frame (X forward, Y left, Z up).
    """
    pc = vehicle.get_physics_control()
    w  = pc.wheels

    fl_x, fl_y, _ = _wheel_xyz(w[0])
    fr_x, fr_y, _ = _wheel_xyz(w[1])
    rl_x, rl_y, _ = _wheel_xyz(w[2])
    rr_x, rr_y, _ = _wheel_xyz(w[3])

    front_x   = np.mean([fl_x, fr_x]) / 100.0   # cm -> m
    rear_x    = np.mean([rl_x, rr_x]) / 100.0
    wheelbase = abs(front_x - rear_x)

    track_front = abs(fl_y - fr_y) / 100.0       # cm -> m

    # If wheel positions were unavailable (all zero), fall back to bounding box
    if wheelbase < 0.01:
        bb        = vehicle.bounding_box.extent    # half-extents in metres
        wheelbase = round(bb.x * 1.5, 3)          # heuristic: ~75% of body length
        print(f"[warn] Wheelbase estimated from bounding box: {wheelbase:.3f} m  "
              f"(bb.x={bb.x:.3f} m) — verify against vehicle spec sheet")
    if track_front < 0.01:
        bb          = vehicle.bounding_box.extent
        track_front = round(bb.y * 1.6, 3)        # heuristic: ~80% of body width
        print(f"[warn] Track estimated from bounding box: {track_front:.3f} m  "
              f"(bb.y={bb.y:.3f} m) — verify against vehicle spec sheet")

    max_steer_deg = w[0].max_steer_angle           # degrees — front wheels only

    params = dict(
        wheelbase_m   = round(wheelbase, 3),
        track_front_m = round(track_front, 3),
        max_steer_deg = round(max_steer_deg, 2),
        mass_kg       = round(pc.mass, 1),
    )
    print(f"[vehicle] L={wheelbase:.3f}m  track={track_front:.3f}m  "
          f"delta_max={max_steer_deg:.1f}deg  m={pc.mass:.0f}kg")
    return params


# ==============================================================================
# IMU PARSING
# ==============================================================================
def parse_imu(d: "carla.IMUMeasurement", prev_delta_norm: float = 0.0,
              curr_delta_norm: float = 0.0) -> dict:
    """
    Parse one IMU frame into a flat dict.

    Coordinate frame (CARLA body frame, flat ground):
        ax          longitudinal acceleration  [m/s^2]  positive = forward
        ay          lateral acceleration       [m/s^2]  positive = left (ISO 8855)
        gravity     vertical IMU channel       [m/s^2]  ~9.81 at rest (gravity + az_true)
                    NOTE: this is NOT pure vertical acceleration.
                    True vertical accel = gravity - 9.81 m/s^2 on flat ground.
                    Renamed from 'az' to 'gravity' to avoid confusion with vertical dynamics.
        gx          roll rate                  [rad/s]
        gy          pitch rate                 [rad/s]
        psi_dot     yaw rate                   [rad/s]  positive = left-turn (CCW from above)
        delta_dot   steering rate proxy        [rad/s]  d(delta_H)/dt from consecutive samples
                    = (curr_delta_norm - prev_delta_norm) * max_steer_deg / PHYSICS_DT
                    NOTE: normalised steer difference divided by dt — input rate, not torque.

    Post-processing note (for analysts)
        ay_SAE = -ay                  to convert to SAE J670e convention
        az_true = gravity - 9.81     to get true vertical acceleration on flat ground
        Ay sanity check: |ay| > 10 m/s^2 on flat ground is physically impossible;
                         flag the run if this occurs (see ay sanity check in tests).
    """
    delta_dot = ((curr_delta_norm - prev_delta_norm) / PHYSICS_DT)  # normalised/s

    return dict(
        t         = d.timestamp,
        t_wall    = time.time(),         # wall-clock UNIX time, for syncing with seat log
        ax        = d.accelerometer.x,
        ay        = d.accelerometer.y,
        gravity   = d.accelerometer.z,   # renamed from az — see docstring
        gx        = d.gyroscope.x,
        gy        = d.gyroscope.y,
        psi_dot   = d.gyroscope.z,       # yaw rate [rad/s], positive = left turn
        delta_dot = delta_dot,           # steering rate proxy [norm/s] — scale by max_steer_deg
    )


# ==============================================================================
# SHARED UTILITIES
# ==============================================================================
def get_speed_ms(vehicle: carla.Vehicle) -> float:
    """Return vehicle speed [m/s] from velocity vector magnitude."""
    v = vehicle.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2)


def check_ay_sanity(ay_ms2: float, label: str = "") -> None:
    """
    Warn if |ay| > 10 m/s^2 — physically impossible on flat ground.
    Most likely cause: az (gravity channel) leaked into ay due to variable naming error.
    """
    if abs(ay_ms2) > 10.0:
        print(f"  [WARN] |ay| = {ay_ms2:.2f} m/s^2 > 10 m/s^2 {label}")
        print(f"         Check that ay is read from accelerometer.y (not .z).")


def stabilize(sess: SimSession, vehicle: carla.Vehicle,
              target_kmh: float, steer: float = 0.0,
              duration: float = 6.0,
              pi: "PIController | None" = None) -> list:
    """
    Hold vehicle at target_kmh for duration using a PI speed controller.
    steer is a fixed normalised steering input [-1, 1] during stabilisation.

    If a PIController instance is provided it is used and its state preserved
    across calls (useful for continuous ramps). If None, a fresh controller
    is created internally and discarded after the call.

    Returns list of IMU dicts recorded during stabilisation.
    """
    target_ms    = target_kmh / 3.6
    own_pi       = pi is None
    if own_pi:
        pi = PIController(kp=PI_KP, ki=PI_KI, dt=PHYSICS_DT,
                          output_min=0.0, output_max=1.0,
                          integral_max=PI_INTEGRAL_MAX)
    log           = []
    prev_delta    = steer
    for _ in range(int(duration / PHYSICS_DT)):
        spd   = get_speed_ms(vehicle)
        error = target_ms - spd
        thr   = pi.step(error)
        brk   = float(np.clip(-PI_KP * error, 0.0, 1.0)) if error < -1.0 else 0.0
        vehicle.apply_control(carla.VehicleControl(
            throttle=thr, steer=float(steer), brake=brk))
        imu = sess.tick()
        if imu:
            log.append(parse_imu(imu, prev_delta_norm=prev_delta,
                                  curr_delta_norm=steer))
        prev_delta = steer
    return log


def reset_to_spawn(sess: SimSession, vehicle: carla.Vehicle) -> None:
    """
    Teleport vehicle back to its original spawn point, zero velocity,
    hold brake for 60 ticks (~0.6 s) until physics settles.
    """
    sp = sess.spawn_transform
    vehicle.set_transform(sp)
    vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    vehicle.apply_control(carla.VehicleControl(
        throttle=0.0, steer=0.0, brake=1.0, hand_brake=True))
    for _ in range(60):
        sess.tick()
    vehicle.apply_control(carla.VehicleControl(hand_brake=False))
    print(f"  [reset] vehicle at spawn "
          f"({sp.location.x:.1f}, {sp.location.y:.1f})  yaw={sp.rotation.yaw:.1f} deg")


def lateral_deviation_m(vehicle: carla.Vehicle,
                         spawn_transform: carla.Transform) -> float:
    """
    Compute lateral deviation of the vehicle from the spawn heading axis.

    Projects the displacement vector (current_pos - spawn_pos) onto the
    spawn's lateral unit vector (perpendicular to heading).
    Spawn-independent: works regardless of spawn orientation.

    Returns signed lateral offset [m]; positive = left of spawn heading.
    """
    sp_loc = spawn_transform.location
    sp_yaw = math.radians(spawn_transform.rotation.yaw)

    # Spawn heading unit vector (forward)
    fwd_x =  math.cos(sp_yaw)
    fwd_y =  math.sin(sp_yaw)
    # Spawn lateral unit vector (90 deg CCW = left)
    lat_x = -fwd_y
    lat_y =  fwd_x

    loc = vehicle.get_transform().location
    dx  = loc.x - sp_loc.x
    dy  = loc.y - sp_loc.y

    return dx * lat_x + dy * lat_y


def save(vehicle_name: str, test_name: str,
         df: pd.DataFrame, summary: dict,
         out_dir: Path) -> None:
    """Write DataFrame to CSV and summary dict to JSON, timestamped.

    Uses a custom JSON encoder to coerce numpy scalars (np.bool_, np.int64,
    np.float64) to native Python types — Python's stdlib json module rejects
    them otherwise. This commonly bites when a summary field is the result of
    a numpy comparison (e.g. `np.nanmean(...) > 50.0` returns np.bool_).
    """
    def _default(o):
        if isinstance(o, np.bool_):    return bool(o)
        if isinstance(o, np.integer):  return int(o)
        if isinstance(o, np.floating): return float(o)
        if isinstance(o, np.ndarray):  return o.tolist()
        raise TypeError(f"Object of type {type(o).__name__} is not JSON serializable")

    out_dir.mkdir(parents=True, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"{vehicle_name}_{test_name}_{ts}"
    if not df.empty:
        csv_path = out_dir / f"{stem}.csv"
        df.to_csv(csv_path, index=False)
        print(f"  -> {csv_path}")
    json_path = out_dir / f"{stem}_summary.json"
    payload   = {"vehicle": vehicle_name, "timestamp": ts, **summary}
    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2, default=_default)
    print(f"  -> {json_path}")


# ==============================================================================
# AY INCREMENT PRE-VALIDATION (ISO 4138 §8.3)
# ==============================================================================
def validate_ay_increments(radius: float, speed_range_kmh: range,
                            warn_threshold_g: float = AY_INCREMENT_WARN_G) -> None:
    """
    Analytic pre-check: compute expected delta_Ay per speed step for ISO 4138.

    On a constant-radius circle: Ay = v^2 / R
    delta_Ay between adjacent speed steps is deterministic in the feedforward mode.
    In PI mode the actual radius may differ slightly, but the analytic check gives
    the nominal expectation.
    ISO 4138 §8.3 requires data increments <= 0.5 m/s^2 ~ 0.051 g.

    Warns (does not abort) so the operator can reduce step size or increase radius
    before committing to a full data collection run.
    """
    print(f"\n  [ISO 4138 pre-check] Ay increments for R={radius}m")
    prev_ay_g = 0.0
    warned    = False
    for v_kmh in speed_range_kmh:
        v_ms    = v_kmh / 3.6
        ay_g    = (v_ms**2 / radius) / GRAVITY_MS2
        delta_g = ay_g - prev_ay_g
        if delta_g > warn_threshold_g:
            print(f"    {v_kmh:3d} km/h | Ay={ay_g:.4f} g | "
                  f"dAy={delta_g:.4f} g  <- WARN: exceeds ISO §8.3 limit")
            warned = True
        prev_ay_g = ay_g
    if not warned:
        print(f"    All steps within ISO §8.3 limit ({warn_threshold_g:.3f} g)")


# ==============================================================================
# CIRCLE STEER COMPUTATION — feedforward and PI modes
# ==============================================================================
def circle_steer(
    speed_ms: float,
    radius: float,
    wheelbase_m: float,
    max_steer_deg: float,
    direction_sign: float,          # +1 = left turn, -1 = right turn
    psi_dot_meas: float,            # measured yaw rate [rad/s]
    circle_pi: "PIController",
    mode: str,                      # "pi" or "feedforward"
) -> float:
    """
    Compute normalised steer command [-1, 1] to hold a circle of given radius.

    feedforward mode
    ----------------
    Pure Ackermann: delta = atan(L / R).
    No feedback. Simple, predictable, may drift at speed.

    pi mode
    -------
    Ackermann feed-forward + PI correction on RADIUS error.

    Sign convention (this is the critical bit — see the global constants block):
        R_meas  = v / |psi_dot_meas|                       (radius from IMU)
        R_error = direction_sign * (R_meas - R_target) / R_target   (RELATIVE)
            R_meas > R_target (wandering out) -> positive error -> MORE steer
            R_meas < R_target (cutting in)    -> negative error -> LESS steer
        The error is dimensionless (fraction of target radius), so one gain set
        holds at any radius — see v8 note in the constants block.

    When |psi_dot_meas| is near zero (vehicle still accelerating onto circle),
    R_meas is unreliable; the correction is gated off below 0.01 rad/s.

    The PI output is added to the Ackermann feed-forward and then clamped to
    the full normalised steer range [-1, 1]. The PI's internal integral
    accumulator is bounded separately by CIRCLE_INTEGRAL_MAX (anti-windup) so
    that a large transient error can still command full steer if needed.

    Steer linearity assumption
    --------------------------
    CARLA maps normalised steer [-1, 1] linearly to [-max_steer_deg, +max_steer_deg].
    This is a simulator-level approximation consistent with the VDxVC protocol
    (relative validity between vehicles, not absolute physical steer angles).
    """
    delta_ack_deg  = math.degrees(math.atan(wheelbase_m / radius))
    delta_ack_norm = float(np.clip(
        direction_sign * delta_ack_deg / max_steer_deg, -1.0, 1.0))

    if mode == "feedforward":
        return delta_ack_norm

    # pi mode: add radius-error correction
    if abs(psi_dot_meas) > 0.01:
        R_meas  = abs(speed_ms / psi_dot_meas)
        # RELATIVE error (dimensionless): positive => steer MORE to tighten.
        # Normalising by the target radius makes CIRCLE_KP/KI radius-independent.
        R_error = direction_sign * (R_meas - radius) / radius
    else:
        R_error = 0.0   # gate off correction until vehicle is on the circle

    steer_corr = circle_pi.step(R_error)
    return float(np.clip(delta_ack_norm + steer_corr, -1.0, 1.0))


# ==============================================================================
# TEST 1 — ISO 4138: STEADY-STATE CIRCULAR (METHOD 1 — CONSTANT RADIUS)
# ==============================================================================
def _run_iso4138_direction(
    sess: SimSession,
    vehicle: carla.Vehicle,
    params: dict,
    radius: float,
    direction: str,          # "left" or "right"
    circle_mode: str,        # "pi" or "feedforward"
) -> tuple:
    """
    Run one directional pass of ISO 4138 Method 1 (constant-radius speed ramp).

    Direction sign convention:
        left  : positive yaw rate (CCW), positive Ackermann steer
        right : negative yaw rate (CW),  negative Ackermann steer

    Returns (df_plateau, df_raw, plateau_rows_list).
    """
    direction_sign = +1.0 if direction == "left" else -1.0
    L              = params["wheelbase_m"]
    max_steer_deg  = params["max_steer_deg"]

    speed_pi  = PIController(kp=PI_KP, ki=PI_KI, dt=PHYSICS_DT,
                             output_min=0.0, output_max=1.0,
                             integral_max=PI_INTEGRAL_MAX)
    circle_pi = PIController(kp=CIRCLE_KP, ki=CIRCLE_KI, dt=PHYSICS_DT,
                             output_min=-1.0, output_max=1.0,
                             integral_max=CIRCLE_INTEGRAL_MAX)

    plateau_rows = []
    raw_rows     = []
    speed_range  = range(20, 102, 2)
    prev_steer   = 0.0

    for speed_kmh in speed_range:
        speed_ms = speed_kmh / 3.6
        circle_pi.reset()
        imu_log  = []

        for _ in range(int(5.0 / PHYSICS_DT)):   # 5 s plateau per step
            spd     = get_speed_ms(vehicle)
            spd_err = speed_ms - spd
            thr     = speed_pi.step(spd_err)
            brk     = float(np.clip(-PI_KP * spd_err, 0.0, 1.0)) \
                      if spd_err < -1.0 else 0.0

            imu_frame  = sess.tick()
            psi_dot_m  = imu_frame.gyroscope.z if imu_frame else 0.0

            steer = circle_steer(
                speed_ms       = spd,
                radius         = radius,
                wheelbase_m    = L,
                max_steer_deg  = max_steer_deg,
                direction_sign = direction_sign,
                psi_dot_meas   = psi_dot_m,
                circle_pi      = circle_pi,
                mode           = circle_mode,
            )

            vehicle.apply_control(carla.VehicleControl(
                throttle=thr, steer=steer, brake=brk))

            if imu_frame:
                row = parse_imu(imu_frame,
                                prev_delta_norm=prev_steer,
                                curr_delta_norm=steer)
                row["speed_kmh"]  = speed_kmh
                row["steer_norm"] = steer
                row["SWA_deg"]    = steer * max_steer_deg
                row["direction"]  = direction
                # Log instantaneous measured radius for diagnostics
                row["R_meas_m"]   = (abs(spd / psi_dot_m)
                                     if abs(psi_dot_m) > 0.01 else float("nan"))
                imu_log.append(row)
                prev_steer = steer

        raw_rows.extend(imu_log)
        if len(imu_log) < 100:
            continue

        # Plateau: mean of last 2 s (200 samples at 100 Hz)
        plateau  = imu_log[-200:]
        ay_mean  = float(np.mean([r["ay"]      for r in plateau]))
        gz_mean  = float(np.mean([r["psi_dot"] for r in plateau]))
        swa_mean = float(np.mean([r["SWA_deg"] for r in plateau]))
        r_mean   = float(np.nanmean([r["R_meas_m"] for r in plateau]))

        check_ay_sanity(ay_mean, label=f"ISO4138 {direction} {speed_kmh}km/h plateau")

        plateau_rows.append(dict(
            direction    = direction,
            speed_kmh    = speed_kmh,
            Ay_ms2       = ay_mean,
            Ay_g         = ay_mean / GRAVITY_MS2,
            psi_dot_rads = gz_mean,
            SWA_deg      = swa_mean,
            R_meas_m     = round(r_mean, 2),
        ))
        print(f"    {direction:5s} {speed_kmh:3d} km/h | "
              f"Ay={ay_mean/GRAVITY_MS2:+.3f} g | "
              f"psi_dot={gz_mean:+.4f} rad/s | "
              f"SWA={swa_mean:.2f} deg | "
              f"R_meas={r_mean:.1f} m")

        # Early stop: Ay plateau -> lateral limit reached
        if len(plateau_rows) >= 3:
            ay_recent = [r["Ay_g"] for r in plateau_rows[-3:]]
            if (abs(ay_recent[-1]) > 0.1 and
                    (max(ay_recent) - min(ay_recent)) < AY_PLATEAU_THRESHOLD_G):
                print(f"    -> Ay plateau reached, stopping {direction} ramp")
                break

    return pd.DataFrame(plateau_rows), pd.DataFrame(raw_rows), plateau_rows


def test_iso4138(sess: SimSession, vehicle: carla.Vehicle,
                 params: dict, radius: float = 50.0,
                 circle_mode: str = "pi") -> tuple:
    """
    ISO 4138:2021 — Steady-state circular driving behaviour.
    Method 1: Constant radius (ISO §8.4).

    Protocol
    --------
    Both directions (left and right) are run sequentially.
    Speed is ramped from 20 to 100 km/h in 2 km/h steps.
    At each step the vehicle is held for 5 s; the last 2 s are averaged
    for the plateau value (steady-state condition).
    Results are pooled across directions for K regression.

    Circle-holding modes (--circle-control)
    ----------------------------------------
    'pi'          PI on radius error R_meas - R_target.
                  R_meas = v / psi_dot_meas (instantaneous, from IMU).
                  Ackermann feed-forward + PI correction. Corrects the
                  radius overshoot observed with yaw-rate PI at speed.
                  Correction gated off when psi_dot < 0.01 rad/s.
    'feedforward' Pure Ackermann steer = atan(L / R). No feedback.
                  May produce larger-than-target radius at speed.
                  Use to compare against PI mode diagnostically.

    Primary outputs
    ---------------
    K_left, K_right  Understeer gradient per direction [deg/g]
    K_pooled         Pooled (mean of both directions) [deg/g]
    R_meas_mean_m    Mean measured radius per direction (diagnostic)

    delta_dot logged as steering rate proxy (d(delta_norm)/dt, scale by max_steer_deg).
    """
    print(f"\n{'─'*60}")
    print(f"[ISO 4138 Method 1] R={radius}m | mode={circle_mode}")

    validate_ay_increments(radius, range(20, 102, 2))

    all_plateau = []
    all_raw     = []

    for direction in ("left", "right"):
        print(f"\n  -- Direction: {direction} --")
        reset_to_spawn(sess, vehicle)
        # Brief straight-line run to get onto circle
        stabilize(sess, vehicle, 20.0, steer=0.0, duration=3.0)

        df_p, df_r, rows = _run_iso4138_direction(
            sess, vehicle, params, radius, direction, circle_mode)
        all_plateau.append(df_p)
        all_raw.append(df_r)

    df_plateau = pd.concat(all_plateau, ignore_index=True)
    df_raw     = pd.concat(all_raw,     ignore_index=True)

    if df_plateau.empty:
        print("  [warn] No plateau data collected.")
        return df_plateau, df_raw, {"test": "iso4138", "error": "no data"}

    # Understeer gradient K per direction and pooled.
    # v8: fit only the clean linear window [AY_FIT_MIN_G, AY_FIT_MAX_G] and drop
    # plateaus where the measured radius drifted outside R_TOLERANCE_FRAC of the
    # target. Both guard against the non-monotonic transition and handling-limit
    # plateaus that made K_left and K_right disagree in v7.
    def fit_K(df_dir):
        ay  = df_dir["Ay_g"].abs()
        rok = (df_dir["R_meas_m"] - radius).abs() <= R_TOLERANCE_FRAC * radius
        lin = df_dir[(ay >= AY_FIT_MIN_G) & (ay <= AY_FIT_MAX_G) & rok]
        n   = len(lin)
        if n < 3:
            return float("nan"), float("nan"), n
        x = lin["Ay_g"].abs().to_numpy()
        y = lin["SWA_deg"].abs().to_numpy()
        slope = float(np.polyfit(x, y, 1)[0])          # deg / g — K is the slope
        r2    = float(np.corrcoef(x, y)[0, 1] ** 2) if n > 2 else float("nan")
        return slope, r2, n

    df_left  = df_plateau[df_plateau["direction"] == "left"]
    df_right = df_plateau[df_plateau["direction"] == "right"]
    K_left,  R2_left,  n_left_fit  = fit_K(df_left)
    K_right, R2_right, n_right_fit = fit_K(df_right)
    K_pooled = float(np.nanmean([K_left, K_right]))

    # L/R asymmetry check — a large spread flags steering trim / vehicle asymmetry
    k_asym = (abs(K_left - K_right) / abs(K_pooled)
              if K_pooled not in (0.0, float("nan")) and not math.isnan(K_pooled)
              else float("nan"))
    k_asymmetric = (not math.isnan(k_asym)) and k_asym > K_ASYMMETRY_WARN_FRAC
    if k_asymmetric:
        print(f"  [WARN] K_left ({K_left:.2f}) and K_right ({K_right:.2f}) differ "
              f"by {k_asym*100:.0f}% — check steering trim / front-rear balance.")

    summary = dict(
        test                            = "iso4138",
        method                          = "constant_radius_M1",
        circle_control_mode             = circle_mode,
        radius_target_m                 = radius,
        radius_meas_mean_left_m         = round(float(df_left["R_meas_m"].mean()), 2)
                                          if not df_left.empty else None,
        radius_meas_mean_right_m        = round(float(df_right["R_meas_m"].mean()), 2)
                                          if not df_right.empty else None,
        understeer_gradient_K_left      = round(K_left,   2),
        understeer_gradient_K_right     = round(K_right,  2),
        understeer_gradient_K_pooled    = round(K_pooled, 2),
        K_fit_R2_left                   = round(R2_left,  3) if not math.isnan(R2_left)  else None,
        K_fit_R2_right                  = round(R2_right, 3) if not math.isnan(R2_right) else None,
        K_fit_window_g                  = [AY_FIT_MIN_G, AY_FIT_MAX_G],
        K_fit_n_left                    = n_left_fit,
        K_fit_n_right                   = n_right_fit,
        K_left_right_asymmetric         = bool(k_asymmetric),
        max_Ay_g                        = round(float(df_plateau["Ay_g"].abs().max()), 3),
        n_plateaus_left                 = len(df_left),
        n_plateaus_right                = len(df_right),
        circle_hold_kp                  = CIRCLE_KP,
        circle_hold_ki                  = CIRCLE_KI,
        spawn_x                         = round(sess.spawn_transform.location.x, 2),
        spawn_y                         = round(sess.spawn_transform.location.y, 2),
        spawn_yaw_deg                   = round(sess.spawn_transform.rotation.yaw, 2),
    )
    print(f"\n  [ISO 4138 result]  K_left={K_left:.2f}  K_right={K_right:.2f}  "
          f"K_pooled={K_pooled:.2f} deg/g")
    print(f"  R_meas  left={summary['radius_meas_mean_left_m']} m  "
          f"right={summary['radius_meas_mean_right_m']} m  (target={radius} m)")
    return df_plateau, df_raw, summary


# ==============================================================================
# TEST 2 — ISO 7401: LATERAL TRANSIENT RESPONSE (STEP STEER)
# ==============================================================================
def _run_iso7401_single(
    sess: SimSession,
    vehicle: carla.Vehicle,
    params: dict,
    speed_kmh: float,
    steer_amplitude: float,     # normalised [-1, 1]
    direction_sign: float,      # +1 left, -1 right
    rise_time_s: float,
) -> dict:
    """
    Execute one ISO 7401 step-steer run and return scalar results.

    Returns a dict with T90_ms_ISO, T90_ms_onset, yaw_gain, ay_ss, psi_dot_ss.
    Returns None if data is insufficient.

    T90 reference point
    -------------------
    ISO 7401 §10.2.1: T90 measured from the moment the steer input is 50% complete
    (= onset_t + rise_time_s / 2).
    Both ISO-compliant (T90_ms_ISO) and onset-referenced (T90_ms_onset) are returned.

    Note on T90 and human perception (VDxVC context)
    -------------------------------------------------
    Human perceptual JND for T90 differences ~ 100-150 ms (Melzi & Previati 2025).
    Absolute T90 values are less informative than delta_T90 between vehicles.
    delta_T90 is unaffected by the 50%-reference offset since it is constant and
    cancels in the difference. Both values logged for analyst flexibility.

    Steer linearity assumption
    --------------------------
    CARLA maps normalised steer [-1, 1] linearly to physical steer angle.
    This is a simulator-level approximation. See module docstring.
    """
    PRE_S   = 0.5    # pre-step baseline recording [s]
    HOLD_S  = 3.0    # post-step hold [s]
    TOTAL_S = PRE_S + rise_time_s + HOLD_S

    amp     = direction_sign * steer_amplitude
    rows    = []
    onset_t = None
    prev_st = 0.0

    speed_pi = PIController(kp=PI_KP, ki=PI_KI, dt=PHYSICS_DT,
                            output_min=0.0, output_max=1.0,
                            integral_max=PI_INTEGRAL_MAX)

    for i in range(int(TOTAL_S / PHYSICS_DT)):
        t_rel = i * PHYSICS_DT

        # Steer profile: flat -> linear ramp -> hold
        if t_rel < PRE_S:
            steer = 0.0
        elif t_rel < PRE_S + rise_time_s:
            steer = amp * ((t_rel - PRE_S) / rise_time_s)
            if onset_t is None:
                onset_t = t_rel
        else:
            steer = amp

        spd = get_speed_ms(vehicle)
        err = speed_kmh / 3.6 - spd
        thr = speed_pi.step(err)
        brk = float(np.clip(-PI_KP * err, 0.0, 1.0)) if err < -1.0 else 0.0
        vehicle.apply_control(carla.VehicleControl(
            throttle=thr, steer=float(steer), brake=brk))

        imu = sess.tick()
        if imu:
            row = parse_imu(imu, prev_delta_norm=prev_st, curr_delta_norm=steer)
            row["t_rel"]       = t_rel
            row["steer_input"] = steer
            rows.append(row)
        prev_st = steer

    if not rows or onset_t is None:
        return None, []

    df = pd.DataFrame(rows)

    # Steady state: mean of last 1 s
    psi_dot_ss = float(df[df["t_rel"] > TOTAL_S - 1.0]["psi_dot"].mean())
    ay_ss      = float(df[df["t_rel"] > TOTAL_S - 1.0]["ay"].mean())

    check_ay_sanity(ay_ss, label=f"ISO7401 dir={'L' if direction_sign>0 else 'R'}")

    # T90 — ISO reference = step 50% complete
    ref_t    = onset_t + rise_time_s / 2.0
    post     = df[df["t_rel"] > ref_t]
    t90_rows = post[post["psi_dot"].abs() >= 0.9 * abs(psi_dot_ss)]
    T90_ISO  = float(t90_rows.iloc[0]["t_rel"] - ref_t) * 1000 \
               if not t90_rows.empty else float("nan")

    # T90 onset-referenced (raw, for analyst cross-check)
    t90_onset_rows = df[(df["t_rel"] > onset_t) &
                        (df["psi_dot"].abs() >= 0.9 * abs(psi_dot_ss))]
    T90_onset = float(t90_onset_rows.iloc[0]["t_rel"] - onset_t) * 1000 \
                if not t90_onset_rows.empty else float("nan")

    # Yaw rate gain: psi_dot_ss / delta_wheel_rad
    # Steer linearity assumed — see docstring
    steer_rad = math.radians(steer_amplitude * params["max_steer_deg"])
    yaw_gain  = abs(psi_dot_ss) / steer_rad if steer_rad > 0 else float("nan")

    result = dict(
        T90_ms_ISO   = round(T90_ISO,  1),
        T90_ms_onset = round(T90_onset, 1),
        yaw_gain     = round(yaw_gain, 4),
        psi_dot_ss   = round(psi_dot_ss, 4),
        ay_ss_ms2    = round(ay_ss, 3),
        ay_ss_g      = round(ay_ss / GRAVITY_MS2, 3),
    )
    return result, rows


def _calibrate_swa_for_ay(
    sess: SimSession,
    vehicle: carla.Vehicle,
    params: dict,
    speed_kmh: float,
    ay_target_ms2: float,
    direction: str,           # "left" or "right"
) -> "dict | None":
    """
    ISO 7401 §10.1 amplitude-determination procedure (v8: closed-loop on Ay).

    Steers the vehicle at constant speed and adjusts the steering input with a
    PI controller until the MEASURED lateral acceleration reaches the target,
    then returns the mean steering-wheel angle held during the plateau. This SWA
    is the input amplitude for the subsequent step manoeuvre.

    Why closed-loop on Ay (not a kinematic radius)
    ----------------------------------------------
    v7 held an open-loop circle of radius R = v^2 / Ay_target. At high speed the
    required steer for 2-4 m/s^2 is a fraction of a degree, below what the
    open-loop circle could hold, so the car ran almost straight and produced
    Ay roughly two orders of magnitude below target. Driving the steer from the
    Ay error removes that failure mode: the controller simply adds steer until
    the IMU reports the target Ay, whatever radius that implies.

    Cross-check
    -----------
    Returns K_at_target = SWA_deg / Ay_g, which should agree with the ISO 4138
    K_pooled within scatter. A discrepancy flags non-linearity at this Ay or a
    controller-tuning issue.

    Returns
    -------
    dict with:
        swa_deg            mean SWA during the plateau [deg]
        swa_norm           normalised steer input [-1, 1]
        ay_ss_g            measured plateau Ay [g]
        ay_ss_ms2          measured plateau Ay [m/s^2]
        R_target_m         kinematic target radius v^2/Ay (diagnostic only)
        R_meas_m           measured radius v / psi_dot during plateau
        K_at_target        understeer gradient at this point [deg/g]
        ay_target_reached  True if |Ay - target| < AY_CAL_TOL_G at the plateau
    Returns None if no usable plateau is collected.
    """
    direction_sign = +1.0 if direction == "left" else -1.0
    speed_ms       = speed_kmh / 3.6
    ay_target_g    = ay_target_ms2 / GRAVITY_MS2
    R_target       = speed_ms ** 2 / ay_target_ms2   # diagnostic reference only
    L              = params["wheelbase_m"]
    max_steer_deg  = params["max_steer_deg"]

    # Speed PI (throttle) and the v8 Ay PI (steer).
    speed_pi  = PIController(kp=PI_KP, ki=PI_KI, dt=PHYSICS_DT,
                             output_min=0.0, output_max=1.0,
                             integral_max=PI_INTEGRAL_MAX)
    ay_pi     = PIController(kp=AY_CAL_KP, ki=AY_CAL_KI, dt=PHYSICS_DT,
                             output_min=0.0, output_max=1.0,
                             integral_max=AY_CAL_INT_MAX)

    # Get vehicle moving straight at target speed first
    stabilize(sess, vehicle, speed_kmh, steer=0.0, duration=4.0, pi=speed_pi)

    # Then ramp steer via the Ay controller for 8 s — last 2 s averaged.
    # 6 s of settling is enough for the integral to reach the steer the target
    # demands at any of the 2/4/6 m/s^2 levels; the final 2 s is the plateau.
    HOLD_S       = 8.0
    PLATEAU_S    = 2.0
    samples      = []

    for i in range(int(HOLD_S / PHYSICS_DT)):
        spd       = get_speed_ms(vehicle)
        spd_err   = speed_ms - spd
        thr       = speed_pi.step(spd_err)
        brk       = float(np.clip(-PI_KP * spd_err, 0.0, 1.0)) \
                    if spd_err < -1.0 else 0.0

        imu_frame = sess.tick()
        ay_meas   = imu_frame.accelerometer.y if imu_frame else 0.0
        psi_dot_m = imu_frame.gyroscope.z     if imu_frame else 0.0

        # Steer magnitude from |Ay| error. BOTH sides must be in g: ay_meas comes
        # from the IMU in m/s^2, so convert before differencing against the
        # g-valued target. (v8.0 compared g against m/s^2, so the loop converged
        # to ay = target/g — i.e. 9.81x too small. The controller itself was
        # correct; only the error units were wrong.)
        ay_meas_g = abs(ay_meas) / GRAVITY_MS2
        ay_err    = ay_target_g - ay_meas_g
        steer_mag = ay_pi.step(ay_err)
        steer     = float(np.clip(direction_sign * steer_mag, -1.0, 1.0))

        vehicle.apply_control(carla.VehicleControl(
            throttle=thr, steer=steer, brake=brk))

        if imu_frame:
            samples.append(dict(
                t_rel     = i * PHYSICS_DT,
                ay        = ay_meas,
                psi_dot   = psi_dot_m,
                steer     = steer,
                speed_ms  = spd,
            ))

    if not samples:
        return None

    # Plateau = last PLATEAU_S seconds
    cutoff  = HOLD_S - PLATEAU_S
    plateau = [s for s in samples if s["t_rel"] >= cutoff]
    if len(plateau) < 50:
        return None

    swa_norm_mean = float(np.mean([s["steer"]   for s in plateau]))
    ay_ss         = float(np.mean([s["ay"]      for s in plateau]))
    psi_dot_ss    = float(np.mean([s["psi_dot"] for s in plateau]))
    R_meas        = (abs(speed_ms / psi_dot_ss)
                     if abs(psi_dot_ss) > 0.01 else float("nan"))

    check_ay_sanity(ay_ss, label=f"ISO7401 calibration {direction} Ay={ay_target_ms2}")

    swa_deg     = swa_norm_mean * max_steer_deg
    ay_ss_g     = ay_ss / GRAVITY_MS2
    K_at_target = (abs(swa_deg) / abs(ay_ss_g)) if abs(ay_ss_g) > 0.01 else float("nan")
    reached     = abs(abs(ay_ss_g) - ay_target_g) < AY_CAL_TOL_G

    return dict(
        swa_deg           = round(swa_deg, 3),
        swa_norm          = round(swa_norm_mean, 5),
        ay_ss_ms2         = round(ay_ss, 3),
        ay_ss_g           = round(ay_ss_g, 3),
        R_target_m        = round(R_target, 2),
        R_meas_m          = round(R_meas, 2) if not math.isnan(R_meas) else None,
        K_at_target       = round(K_at_target, 2) if not math.isnan(K_at_target) else None,
        ay_target_reached = bool(reached),
    )


def test_iso7401(sess: SimSession, vehicle: carla.Vehicle,
                 params: dict,
                 speed_kmh:   float = 100.0,
                 rise_time_s: float = 0.15) -> tuple:
    """
    ISO 7401:2011 — Lateral transient response (open-loop step steer).

    Protocol
    --------
    For each target lateral acceleration in ISO7401_AY_TARGETS_MS2 (ISO §10.1
    defaults: 2, 4, 6 m/s^2), and for each direction (left / right):

      1. CALIBRATE the step amplitude per ISO §10.1: drive the vehicle in a
         steady-state circle of radius R = v^2 / Ay_target at the test speed,
         and measure the steady-state SWA required to hold it. That SWA becomes
         the step amplitude for this (Ay, direction) pair.
         This replaces the previous neutral-steer formula
         delta = Ay * L / v^2, which underestimated the required SWA by a
         factor of approximately (steering_ratio x (1 + understeer correction))
         and produced measured Ay_ss two orders of magnitude below target.
         See _calibrate_swa_for_ay() for details.

      2. STEP RUN x ISO7401_N_REPS using the calibrated SWA. Each rep:
         stabilise on a straight, then ramp the steer to the calibrated
         amplitude over rise_time_s and hold for 3 s.

    Output structure mirrors per-direction (mean +/- SD across reps) plus a
    pooled-across-directions summary. The calibration result (including
    K_at_target as a cross-check against the ISO 4138 K) is logged for each
    (Ay, direction) pair.

    delta_dot logged as proxy (d(delta_norm)/dt * max_steer_deg = approx steering rate).
    """
    print(f"\n{'─'*60}")
    print(f"[ISO 7401] speed={speed_kmh} km/h | rise={rise_time_s*1000:.0f} ms | "
          f"{ISO7401_N_REPS} reps per direction per amplitude")

    speed_ms = speed_kmh / 3.6
    L        = params["wheelbase_m"]

    all_rows  = []
    amplitude_results = {}

    for ay_target in ISO7401_AY_TARGETS_MS2:
        print(f"\n  -- Ay target = {ay_target} m/s^2 --")
        dir_results       = {}
        calibration_data  = {}

        for direction, sign in (("left", +1.0), ("right", -1.0)):
            # ── STEP 1: calibrate SWA via steady-state circle (ISO §10.1) ────
            reset_to_spawn(sess, vehicle)
            print(f"    [{direction}] calibrating SWA via "
                  f"R={speed_ms**2/ay_target:.1f} m circle ...")
            calib = _calibrate_swa_for_ay(
                sess, vehicle, params,
                speed_kmh      = speed_kmh,
                ay_target_ms2  = ay_target,
                direction      = direction,
            )
            if calib is None:
                print(f"    [{direction}] calibration FAILED — skipping")
                continue

            print(f"    [{direction}] SWA={calib['swa_deg']:+.2f} deg "
                  f"(norm={calib['swa_norm']:+.4f}) | "
                  f"plateau Ay={calib['ay_ss_g']:+.3f} g "
                  f"(target {ay_target/GRAVITY_MS2:.3f} g) | "
                  f"reached={calib['ay_target_reached']} | "
                  f"R_meas={calib['R_meas_m']} m | "
                  f"K_at_target={calib['K_at_target']} deg/g")
            if not calib["ay_target_reached"]:
                print(f"    [{direction}] [WARN] Ay calibration did not reach "
                      f"target within {AY_CAL_TOL_G} g — step amplitude may be off.")

            calibration_data[direction] = calib
            # Use the magnitude of the calibrated norm as the step amplitude;
            # direction_sign in _run_iso7401_single applies the turn direction.
            delta_norm = abs(calib["swa_norm"])

            # ── STEP 2: step-steer reps using the calibrated amplitude ───────
            rep_results = []
            for rep in range(1, ISO7401_N_REPS + 1):
                reset_to_spawn(sess, vehicle)
                stabilize(sess, vehicle, speed_kmh, steer=0.0, duration=8.0)
                result, rows = _run_iso7401_single(
                    sess, vehicle, params,
                    speed_kmh       = speed_kmh,
                    steer_amplitude = delta_norm,
                    direction_sign  = sign,
                    rise_time_s     = rise_time_s,
                )
                if result is None:
                    print(f"      rep {rep}: no data")
                    continue
                for r in rows:
                    r["direction"]       = direction
                    r["rep"]             = rep
                    r["ay_target"]       = ay_target
                    r["swa_calib_deg"]   = calib["swa_deg"]
                all_rows.extend(rows)
                rep_results.append(result)
                print(f"      rep {rep}: T90={result['T90_ms_ISO']:.1f} ms | "
                      f"gain={result['yaw_gain']:.4f} | "
                      f"ay_ss={result['ay_ss_g']:.3f} g")

            if rep_results:
                T90s   = [r["T90_ms_ISO"] for r in rep_results]
                gains  = [r["yaw_gain"]   for r in rep_results]
                ays    = [r["ay_ss_g"]    for r in rep_results]
                dir_results[direction] = dict(
                    swa_calib_deg     = calib["swa_deg"],
                    swa_calib_norm    = calib["swa_norm"],
                    R_target_m        = calib["R_target_m"],
                    R_meas_calib_m    = calib["R_meas_m"],
                    K_at_target       = calib["K_at_target"],
                    ay_ss_calib_g     = calib["ay_ss_g"],
                    ay_target_reached = calib["ay_target_reached"],
                    ay_ss_step_mean_g = round(float(np.nanmean(ays)),   4),
                    T90_ms_mean       = round(float(np.nanmean(T90s)),  1),
                    T90_ms_sd         = round(float(np.nanstd(T90s)),   1),
                    gain_mean         = round(float(np.nanmean(gains)), 4),
                    gain_sd           = round(float(np.nanstd(gains)),  4),
                    n_reps            = len(rep_results),
                )

        # Pooled across directions
        all_T90  = [d["T90_ms_mean"] for d in dir_results.values()]
        all_gain = [d["gain_mean"]   for d in dir_results.values()]

        amplitude_results[f"Ay_{ay_target}ms2"] = dict(
            ay_target_ms2          = ay_target,
            per_direction          = dir_results,
            T90_ms_pooled_mean     = round(float(np.nanmean(all_T90)),  1)
                                     if all_T90  else None,
            yaw_gain_pooled_mean   = round(float(np.nanmean(all_gain)), 4)
                                     if all_gain else None,
            # Response discriminable: T90 > 50 ms (Melzi & Previati 2025 threshold).
            # For cross-vehicle discrimination, use delta_T90 > 50 ms.
            response_discriminable = (bool(all_T90)
                                      and not math.isnan(np.nanmean(all_T90))
                                      and np.nanmean(all_T90) > 50.0),
        )

    summary = dict(
        test                   = "iso7401",
        speed_kmh              = speed_kmh,
        rise_time_ms           = rise_time_s * 1000,
        n_reps_per_direction   = ISO7401_N_REPS,
        amplitude_results      = amplitude_results,
        spawn_x                = round(sess.spawn_transform.location.x, 2),
        spawn_y                = round(sess.spawn_transform.location.y, 2),
        spawn_yaw_deg          = round(sess.spawn_transform.rotation.yaw, 2),
    )

    print(f"\n  [ISO 7401 summary]")
    for amp_key, res in amplitude_results.items():
        print(f"    {amp_key}: T90_pooled={res['T90_ms_pooled_mean']} ms | "
              f"gain_pooled={res['yaw_gain_pooled_mean']:.4f} | "
              f"discriminable={res['response_discriminable']}")

    return pd.DataFrame(all_rows), summary


# ==============================================================================
# TEST 3 — THROTTLE STEP RESPONSE (longitudinal)
# ==============================================================================
def test_throttle_step(sess: SimSession, vehicle: carla.Vehicle,
                       initial_speeds_kmh: list = None) -> tuple:
    """
    Full-throttle step from stable speed at multiple initial conditions.

    No ISO standard number — custom characterization for VDQ Item 1 and Item 5.
    The step is open-loop (full throttle = 1.0) and instantaneous — worst-case jerk.
    For EV vs. ICE discrimination this is the most sensitive manoeuvre.

    Steer is fixed at 0.0 throughout — no steer-linearity assumption required.

    Primary outputs
    ---------------
    peak_ax_ms2   Peak longitudinal acceleration [m/s^2]
    peak_jerk_ms3 Peak jerk d(ax)/dt [m/s^3]  — EV vs. ICE discriminator
    t_peak_ax_s   Time to peak acceleration [s]

    Feeds VDQ a priori predictions:
        Item 1 (throttle response) — rank order: Model X > SLC > Micra
        Item 5 (perceived mass)    — peak_ax / mass correlates inversely
    """
    if initial_speeds_kmh is None:
        initial_speeds_kmh = [30, 60, 80]

    print(f"\n{'─'*60}")
    print(f"[Throttle Step] v0 = {initial_speeds_kmh} km/h")

    all_rows = []
    runs     = {}

    for v0 in initial_speeds_kmh:
        reset_to_spawn(sess, vehicle)
        speed_pi = PIController(kp=PI_KP, ki=PI_KI, dt=PHYSICS_DT,
                                output_min=0.0, output_max=1.0,
                                integral_max=PI_INTEGRAL_MAX)
        stabilize(sess, vehicle, v0, steer=0.0, duration=6.0, pi=speed_pi)
        # v8: brief coast (zero throttle) before the step so every run starts the
        # step from the same powertrain state. Without it the step is applied on
        # top of whatever throttle the speed-PI happened to hold, which put the
        # acceleration peak at frame 0 (t_peak=0) and made the boundary jerk
        # identical across speeds.
        for _ in range(int(0.4 / PHYSICS_DT)):
            vehicle.apply_control(carla.VehicleControl(
                throttle=0.0, steer=0.0, brake=0.0))
            sess.tick()
        print(f"  stabilized @ {v0} km/h — applying full-throttle step")

        run_rows  = []
        prev_st   = 0.0
        for i in range(int(4.0 / PHYSICS_DT)):   # 4 s run
            vehicle.apply_control(carla.VehicleControl(
                throttle=1.0, steer=0.0, brake=0.0))
            imu = sess.tick()
            if imu:
                row = parse_imu(imu, prev_delta_norm=prev_st, curr_delta_norm=0.0)
                row["t_rel"]  = i * PHYSICS_DT
                row["v0_kmh"] = v0
                run_rows.append(row)

        all_rows.extend(run_rows)
        if not run_rows:
            continue

        ax_arr   = np.array([r["ax"]    for r in run_rows])
        t_arr    = np.array([r["t_rel"] for r in run_rows])

        # v8: skip the first few samples so the step-onset control discontinuity
        # is not mistaken for the physical peak, and smooth ax with a short
        # moving average before differentiating so jerk reflects the powertrain,
        # not a single-frame numerical spike.
        SETTLE_SKIP = 3                    # samples (~30 ms) after the step edge
        WIN         = 5                    # moving-average window for jerk
        if len(ax_arr) <= SETTLE_SKIP + WIN:
            continue
        ax_s     = ax_arr[SETTLE_SKIP:]
        t_s      = t_arr[SETTLE_SKIP:]
        kernel   = np.ones(WIN) / WIN
        ax_sm    = np.convolve(ax_s, kernel, mode="same")
        jerk_arr = np.gradient(ax_sm, t_s)
        idx_peak = int(np.argmax(ax_s))

        runs[f"{v0}kmh"] = dict(
            peak_ax_ms2   = round(float(ax_s[idx_peak]), 3),
            peak_jerk_ms3 = round(float(jerk_arr.max()), 3),
            t_peak_ax_s   = round(float(t_s[idx_peak] - t_arr[SETTLE_SKIP]), 3),
        )
        print(f"  v0={v0} km/h | ax_peak={ax_s[idx_peak]:.2f} m/s^2 | "
              f"jerk_peak={jerk_arr.max():.2f} m/s^3 | "
              f"t_peak={t_s[idx_peak]-t_arr[SETTLE_SKIP]:.3f} s")

    summary = dict(
        test              = "throttle_step",
        runs              = runs,
        spawn_x           = round(sess.spawn_transform.location.x, 2),
        spawn_y           = round(sess.spawn_transform.location.y, 2),
        spawn_yaw_deg     = round(sess.spawn_transform.rotation.yaw, 2),
    )
    return pd.DataFrame(all_rows), summary


# ==============================================================================
# TEST 4 — ISO 3888-2 PROXY: DOUBLE LANE CHANGE
# ==============================================================================
def test_iso3888(sess: SimSession, vehicle: carla.Vehicle,
                 params: dict,
                 speed_start_kmh: float = 50.0,
                 speed_max_kmh:   float = 110.0,
                 speed_step_kmh:  float = 5.0) -> tuple:
    """
    ISO 3888-2 proxy — sinusoidal open-loop steer at increasing entry speed.

    IMPORTANT: This is NOT a full ISO 3888-2 test.
    A full ISO 3888-2 requires physical gate geometry sized to the vehicle
    bounding box, with pass/fail determined by gate clearance.
    This implementation uses a reproducible open-loop sinusoidal steer profile.
    Results are NOT directly comparable to published ISO 3888-2 gate-pass speeds.
    Use for RELATIVE comparison across vehicles only (same input, different dynamics).

    SDLP is normalised to spawn heading vector — spawn-independent.
    See lateral_deviation_m().

    Primary outputs (relative, vehicle-comparative)
    ------------------------------------------------
    SDLP_m        Lateral position standard deviation [m]
    peak_psi_dot  Peak yaw rate [rad/s]
    peak_Ay_g     Peak lateral acceleration [g]

    Recommended map: flat open map with >200 m clear straight (e.g. Town04).
    """
    print(f"\n{'─'*60}")
    print(f"[ISO 3888-2 proxy] {speed_start_kmh}->{speed_max_kmh} km/h "
          f"step={speed_step_kmh} km/h")
    print("  NOTE: sinusoidal proxy — relative vehicle comparison only")

    STEER_FREQ_HZ     = 0.8    # Hz
    STEER_AMPLITUDE   = 0.4    # normalised
    MANEUVER_DURATION = 5.0    # s per speed level

    all_rows = []
    runs     = {}
    speeds   = np.arange(speed_start_kmh,
                         speed_max_kmh + speed_step_kmh,
                         speed_step_kmh)

    for speed_kmh in speeds:
        reset_to_spawn(sess, vehicle)
        speed_pi = PIController(kp=PI_KP, ki=PI_KI, dt=PHYSICS_DT,
                                output_min=0.0, output_max=1.0,
                                integral_max=PI_INTEGRAL_MAX)
        stabilize(sess, vehicle, float(speed_kmh), steer=0.0,
                  duration=6.0, pi=speed_pi)
        print(f"  running @ {speed_kmh:.0f} km/h")

        run_rows  = []
        lat_devs  = []
        unstable  = False
        prev_st   = 0.0

        for i in range(int(MANEUVER_DURATION / PHYSICS_DT)):
            t_rel = i * PHYSICS_DT
            steer = STEER_AMPLITUDE * math.sin(2 * math.pi * STEER_FREQ_HZ * t_rel)

            spd = get_speed_ms(vehicle)
            err = speed_kmh / 3.6 - spd
            thr = speed_pi.step(err)
            brk = float(np.clip(-PI_KP * err, 0.0, 1.0)) if err < -1.0 else 0.0
            vehicle.apply_control(carla.VehicleControl(
                throttle=thr, steer=float(steer), brake=brk))

            imu = sess.tick()
            if imu:
                row = parse_imu(imu, prev_delta_norm=prev_st, curr_delta_norm=steer)
                row["t_rel"]       = t_rel
                row["steer_input"] = steer
                row["speed_kmh"]   = speed_kmh
                run_rows.append(row)
            prev_st = steer

            lat_devs.append(lateral_deviation_m(vehicle, sess.spawn_transform))

            # Stability check: |psi_dot| > 3 rad/s -> loss of control
            if imu and abs(imu.gyroscope.z) > 3.0:
                print(f"  -> instability @ {speed_kmh:.0f} km/h "
                      f"(|psi_dot| > 3 rad/s)")
                unstable = True
                break

        all_rows.extend(run_rows)
        if not run_rows:
            continue

        df_run   = pd.DataFrame(run_rows)
        sdlp     = float(np.std(lat_devs))
        peak_psi = float(df_run["psi_dot"].abs().max())
        peak_ay  = float(df_run["ay"].abs().max())

        runs[f"{speed_kmh:.0f}kmh"] = dict(
            SDLP_m        = round(sdlp, 4),
            peak_psi_dot  = round(peak_psi, 4),
            peak_Ay_g     = round(peak_ay / GRAVITY_MS2, 3),
            unstable      = unstable,
        )
        print(f"  SDLP={sdlp:.3f}m | psi_dot_peak={peak_psi:.3f} rad/s | "
              f"Ay_peak={peak_ay/GRAVITY_MS2:.3f}g"
              + (" [UNSTABLE]" if unstable else ""))

        if unstable:
            print("  -> stopping speed ramp at instability")
            break

    summary = dict(
        test                 = "iso3888_proxy",
        runs                 = runs,
        steer_freq_hz        = STEER_FREQ_HZ,
        steer_amplitude_norm = STEER_AMPLITUDE,
        spawn_x              = round(sess.spawn_transform.location.x, 2),
        spawn_y              = round(sess.spawn_transform.location.y, 2),
        spawn_yaw_deg        = round(sess.spawn_transform.rotation.yaw, 2),
    )
    return pd.DataFrame(all_rows), summary


# ==============================================================================
# UTILITY: LIST BLUEPRINTS
# ==============================================================================
def list_vehicles(world: carla.World) -> None:
    """Print all vehicle blueprints available on the connected CARLA server."""
    bps = sorted(world.get_blueprint_library().filter("vehicle.*"),
                 key=lambda b: b.id)
    print(f"\n{len(bps)} vehicle blueprints available on server:\n")
    for bp in bps:
        print(f"  {bp.id}")
    print("\nPass any of these directly to --vehicle, e.g.:")
    print("  python vdxvc_characterize.py --vehicle vehicle.nissan.micra")


# ==============================================================================
# MAIN
# ==============================================================================
def main() -> None:
    args = parse_args()

    print(f"\n{'='*60}")
    print("VDxVC Vehicle Dynamics Characterization Suite  v3.0")
    print(f"{'='*60}")

    client = carla.Client(args.host, args.port)
    client.set_timeout(15.0)

    if args.list_vehicles:
        list_vehicles(client.get_world())
        return

    if args.vehicle is None:
        print("Error: --vehicle is required.\n"
              "       Use --list-vehicles to enumerate available blueprints.")
        sys.exit(1)

    # Validate blueprint ID against live server before entering the session
    _world      = client.get_world()
    _all_bp_ids = [bp.id for bp in _world.get_blueprint_library().filter("vehicle.*")]
    if args.vehicle not in _all_bp_ids:
        print(f"\nError: blueprint '{args.vehicle}' not found on server.")
        print("  Available vehicle blueprints:")
        for bp_id in sorted(_all_bp_ids):
            print(f"    {bp_id}")
        print("\nTip: use --list-vehicles for the full list with formatting.")
        sys.exit(1)

    blueprint_id = args.vehicle
    # Output folder from last two dot-segments: "vehicle.nissan.micra" -> "nissan_micra"
    out_folder   = "_".join(blueprint_id.split(".")[-2:])
    out_dir      = Path(args.output) / out_folder

    print(f"Vehicle        : {blueprint_id}")
    print(f"Test           : {args.test}")
    print(f"Circle control : {args.circle_control}")
    print(f"Output         : {out_dir}")
    print(f"Map            : {args.map or '(current)'}")
    print(f"Rendering      : {'on' if args.render else 'off'}")
    print(f"Realtime pacing: {'on (motion platform in the loop)' if args.realtime else 'off'}")
    print(f"{'='*60}\n")

    with SimSession(client, map_name=args.map, render=args.render,
                    realtime=args.realtime) as sess:
        vehicle = sess.spawn_vehicle(blueprint_id)
        params  = get_vehicle_params(vehicle)
        _       = sess.attach_imu(vehicle)

        # Settle physics: 100 ticks ~ 1 s before any test
        print("[sim] Settling physics ...")
        for _ in range(100):
            sess.tick()

        run_all = args.test == "all"

        # -- ISO 4138 ----------------------------------------------------------
        if run_all or args.test == "iso4138":
            reset_to_spawn(sess, vehicle)
            df_p, df_r, smry = test_iso4138(
                sess, vehicle, params,
                radius      = args.radius,
                circle_mode = args.circle_control,
            )
            save(out_folder, "iso4138_plateau", df_p, smry, out_dir)
            save(out_folder, "iso4138_raw",     df_r, {},   out_dir)

        # -- ISO 7401 ----------------------------------------------------------
        if run_all or args.test == "iso7401":
            reset_to_spawn(sess, vehicle)
            df, smry = test_iso7401(sess, vehicle, params)
            save(out_folder, "iso7401", df, smry, out_dir)

        # -- Throttle Step -----------------------------------------------------
        if run_all or args.test == "throttle":
            df, smry = test_throttle_step(sess, vehicle)
            save(out_folder, "throttle_step", df, smry, out_dir)

        # -- ISO 3888-2 proxy --------------------------------------------------
        if run_all or args.test == "iso3888":
            if args.map is None and args.test == "iso3888":
                print("[warn] ISO 3888-2 proxy works best on a flat open map "
                      "(e.g. Town04). Use --map Town04 for a clean straight.")
            reset_to_spawn(sess, vehicle)
            df, smry = test_iso3888(sess, vehicle, params)
            save(out_folder, "iso3888_proxy", df, smry, out_dir)

    print(f"\n{'='*60}")
    print(f"Done. Results in: {out_dir.resolve()}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()