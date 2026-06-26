#!/usr/bin/env python3
"""
ARHUD client 1/3 — ego vehicle (control-driven).

vehicle.nissan.Vr is driven with throttle + steer (real physics, NO teleport /
set_transform). A proportional throttle controller ramps the target speed from
--v-min to --v-max (km/h) over --accel-time, then holds it. Steer is held at a
constant magnitude; its sign flips after each full ~360 deg turn, so the car
circles once left, once right, repeatedly. The live speed/steer is streamed to
the dashboard. Ctrl+C detaches; the car is left in place.
"""
import argparse, logging, math, sys, time
import carla

log = logging.getLogger("c1")


def call(world, m, *a):
    fn = getattr(world, m, None)
    if not fn:
        return
    try:
        fn(*a)
    except Exception as e:  # noqa: BLE001
        log.error("✗ %s: %s", m, e)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--vehicle", default="vehicle.nissan.Vr")
    ap.add_argument("--v-min", type=float, default=20.0, help="start speed (km/h)")
    ap.add_argument("--v-max", type=float, default=140.0, help="top speed (km/h)")
    ap.add_argument("--accel-time", type=float, default=40.0, help="seconds to ramp v-min->v-max")
    ap.add_argument("--steer", type=float, default=0.15, help="steer magnitude (smaller = wider circle)")
    ap.add_argument("--kp", type=float, default=0.06, help="throttle P-gain on speed error")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--no-chase", action="store_true", help="don't move the spectator behind the car")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(message)s", datefmt="%H:%M:%S")

    client = carla.Client(args.host, args.port); client.set_timeout(args.timeout)
    world = client.get_world()
    log.info("Server %s | Client %s", client.get_server_version(), client.get_client_version())
    if not hasattr(world, "set_control_value_dashboard"):
        log.error("World missing set_control_value_dashboard — rebuild PythonAPI."); return 4

    bps = world.get_blueprint_library().filter(args.vehicle)
    if not bps:
        log.error("Blueprint '%s' not found.", args.vehicle); return 2
    sp = world.get_map().get_spawn_points()
    if not sp:
        log.error("No spawn points."); return 3

    vehicle = None
    try:
        vehicle = world.spawn_actor(bps[0], sp[0]); vehicle.set_autopilot(False)
        world.wait_for_tick()
        call(world, "set_type_string_dashboard", "ARHUD")
        log.info("Ego id=%d: driving by control, %.0f->%.0f km/h, steer +/-%.2f. Ctrl+C to detach.",
                 vehicle.id, args.v_min, args.v_max, args.steer)

        spec = world.get_spectator()
        sign = 1.0                       # +1 = left circle, -1 = right circle
        turn_accum = 0.0
        prev_yaw = vehicle.get_transform().rotation.yaw
        t0 = time.time(); last = t0
        while True:
            world.wait_for_tick()
            now = time.time()
            if now - last < args.step:
                time.sleep(0.005); continue
            last = now
            t = now - t0

            # target speed ramp 20 -> 140 km/h, then hold
            frac = min(1.0, t / max(0.1, args.accel_time))
            target = args.v_min + (args.v_max - args.v_min) * frac

            # current speed
            vel = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

            # flip steer sign after each full turn
            yaw = vehicle.get_transform().rotation.yaw
            turn_accum += wrap180(yaw - prev_yaw); prev_yaw = yaw
            if abs(turn_accum) >= 360.0:
                sign = -sign; turn_accum = 0.0
                log.info("Switch to %s circle.", "left" if sign > 0 else "right")

            # throttle / brake P-control toward target speed
            err = target - speed
            throttle = max(0.0, min(1.0, args.kp * err))
            brake = 0.0 if err > -2.0 else max(0.0, min(1.0, -args.kp * err))
            steer = sign * args.steer
            vehicle.apply_control(carla.VehicleControl(
                throttle=throttle, steer=steer, brake=brake))

            # stream live values to the HUD
            call(world, "set_control_value_dashboard", speed, steer)
            call(world, "update_dashboard")

            if not args.no_chase:
                tf = vehicle.get_transform()
                f = tf.get_forward_vector()
                spec.set_transform(carla.Transform(
                    carla.Location(tf.location.x - f.x * 8.0, tf.location.y - f.y * 8.0,
                                   tf.location.z + 4.0),
                    carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)))
    except KeyboardInterrupt:
        log.info("Detaching — ego left in place."); return 0


if __name__ == "__main__":
    sys.exit(main())
