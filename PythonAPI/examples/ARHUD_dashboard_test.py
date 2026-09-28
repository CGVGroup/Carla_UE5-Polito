#!/usr/bin/env python3
"""
Dashboard test client for CGVGroup/Carla_UE5-Polito.

Spawns a `vehicle.nissan.Vr` and exercises every Dashboard RPC via the
World-level bindings (no carla.Dashboard object needed), placing test
elements in front of the ego vehicle, with verbose CLI logging.

Requires the PythonAPI World binding additions:
    world.set_control_value_dashboard(speed, steer)
    world.set_type_string_dashboard(type)
    world.add_vector_pair_dashboard(position, rotation)     # both carla.Vector3D
    world.update_obstacle_dashboard(id, type, isDanger, Vector1, Vector2,
                                    obstacleSpeed, obstacleSteer)
    world.remove_id_obstacle_dashboard(id)
    world.update_dashboard()
    world.trigger_path_dashboard()

Runtime prerequisite: an ADashboard actor must be registered via
ACarlaGameModeBase::SetDashboard(), else every call no-ops silently.
"""

import argparse
import logging
import math
import sys
import time

import carla

log = logging.getLogger("dashboard_test")

DASH_METHODS = (
    "set_type_string_dashboard", "set_control_value_dashboard",
    "add_vector_pair_dashboard", "update_obstacle_dashboard",
    "remove_id_obstacle_dashboard", "update_dashboard", "trigger_path_dashboard",
)


def setup_logging(verbose):
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(message)s",
        datefmt="%H:%M:%S",
    )


def ring_point(center, radius_m, bearing_deg, up_m=0.0):
    """carla.Vector3D on a horizontal ring around `center` (carla.Location).

    bearing_deg is a world-frame compass angle, so points are evenly distributed
    no matter how the camera is rotated.
    """
    a = math.radians(bearing_deg)
    return carla.Vector3D(
        center.x + radius_m * math.cos(a),
        center.y + radius_m * math.sin(a),
        center.z + up_m,
    )


def call(world, method, *args):
    """Invoke a world dashboard method with full logging; never abort the suite."""
    fn = getattr(world, method, None)
    if fn is None:
        log.error("✗ world.%s missing — rebuild PythonAPI with the World bindings.", method)
        return
    pretty = ", ".join(
        f"({a.x:.1f},{a.y:.1f},{a.z:.1f})" if isinstance(a, carla.Vector3D) else repr(a)
        for a in args
    )
    log.debug("→ world.%s(%s)", method, pretty)
    try:
        fn(*args)
        log.info("✓ %s OK", method)
    except Exception as exc:  # noqa: BLE001
        log.error("✗ %s FAILED: %s: %s", method, type(exc).__name__, exc)


def main():
    ap = argparse.ArgumentParser(description="CARLA Dashboard bindings test client")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--vehicle", default="vehicle.nissan.Vr")
    ap.add_argument("--refresh", type=float, default=2.0,
                    help="seconds between idempotent update_dashboard re-pushes (0 = never)")
    ap.add_argument("--count", type=int, default=8, help="obstacles placed around the car")
    ap.add_argument("--radius", type=float, default=12.0, help="ring radius in metres")
    ap.add_argument("--scale", type=float, default=100.0,
                    help="metres->units factor for positions (100 = cm; server "
                         "Vector3D::ToFVector does not scale). Use 1 if you patched "
                         "the server to call ToCentimeters().")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    setup_logging(args.verbose)

    log.info("Connecting to CARLA at %s:%d", args.host, args.port)
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    log.info("Server %s | Client %s", client.get_server_version(), client.get_client_version())

    world = client.get_world()

    missing = [m for m in DASH_METHODS if not hasattr(world, m)]
    if missing:
        log.error("World is missing dashboard bindings: %s", ", ".join(missing))
        log.error("Add the .def lines to PythonAPI/carla/src/World.cpp and rebuild the egg/wheel.")
        return 4
    log.info("All %d World dashboard bindings present.", len(DASH_METHODS))

    bl = world.get_blueprint_library()
    bps = bl.filter(args.vehicle)
    if not bps:
        log.error("Blueprint '%s' not found. Available nissan:", args.vehicle)
        for b in bl.filter("vehicle.nissan.*"):
            log.error("    %s", b.id)
        return 2
    veh_bp = bps[0]

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        log.error("Map has no spawn points.")
        return 3

    vehicle = None
    try:
        vehicle = world.spawn_actor(veh_bp, spawn_points[0])
        vehicle.set_autopilot(False)
        log.info("Spawned ego %s id=%d", veh_bp.id, vehicle.id)

        spec = world.get_spectator()
        spec.set_transform(carla.Transform(
            carla.Location(spawn_points[0].location.x,
                           spawn_points[0].location.y,
                           spawn_points[0].location.z + 2.5),
            carla.Rotation(pitch=-10, yaw=spawn_points[0].rotation.yaw)))

        world.wait_for_tick()
        ego_tf = vehicle.get_transform()
        log.info("Ego loc=(%.1f,%.1f,%.1f) yaw=%.1f",
                 ego_tf.location.x, ego_tf.location.y, ego_tf.location.z, ego_tf.rotation.yaw)

        log.info("──── 1/7 set_type_string_dashboard")
        call(world, "set_type_string_dashboard", "highway")

        log.info("──── 2/7 set_control_value_dashboard (ego speed/steer)")
        v = vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        steer = vehicle.get_control().steer
        log.info("   speed=%.1f km/h steer=%.3f", speed_kmh, steer)
        call(world, "set_control_value_dashboard", float(speed_kmh), float(steer))

        S = args.scale
        center = carla.Vector3D(ego_tf.location.x * S,
                                ego_tf.location.y * S,
                                ego_tf.location.z * S)
        radius = args.radius * S
        n = max(1, args.count)
        step = 360.0 / n
        types = ["car", "pedestrian", "bike", "cone", "sign", "truck", "barrier", "animal"]

        log.info("──── 3/6 add_vector_pair_dashboard (%d waypoints, full ring, scale=%g)", n, S)
        for i in range(n):
            bearing = i * step
            pos = ring_point(center, radius, bearing)
            # pack tangent heading into the rotation Vector3D (.y = yaw)
            call(world, "add_vector_pair_dashboard",
                 pos, carla.Vector3D(0.0, bearing + 90.0, 0.0))

        log.info("──── 4/6 update_obstacle_dashboard (%d obstacles, full ring)", n)
        obstacles = []
        for i in range(n):
            bearing = i * step
            oid = i + 1
            otype = types[i % len(types)]
            v1 = ring_point(center, radius, bearing)               # ground point
            v2 = ring_point(center, radius, bearing, up_m=1.8 * S)  # top point (label)
            o = (oid, otype, bool(i % 2 == 0), v1, v2, 15.0, 0.0)
            obstacles.append(o)
            log.info("   obstacle id=%d type=%-10s bearing=%5.1f° danger=%s",
                     oid, otype, bearing, o[2])
            call(world, "update_obstacle_dashboard", *o)

        log.info("──── 5/6 trigger_path_dashboard")
        call(world, "trigger_path_dashboard")

        log.info("──── 6/6 update_dashboard (push frame)")
        call(world, "update_dashboard")

        log.info("=" * 60)
        log.info("Scene is LIVE: %d obstacles + %d waypoints around the ego.", n, n)
        log.info("Nothing will be removed. Press Ctrl+C to detach the client;")
        log.info("the ego and all dashboard elements stay for study in the editor.")
        log.info("=" * 60)

        # Keep-alive: re-fire OnUpdate only (idempotent). Never re-add obstacles
        # or vector pairs — AddVectorPair APPENDS and would duplicate waypoints.
        last = time.time()
        while True:
            world.wait_for_tick()
            if args.refresh > 0 and (time.time() - last) >= args.refresh:
                call(world, "update_dashboard")
                last = time.time()
            time.sleep(0.05)

    except KeyboardInterrupt:
        log.info("Detaching — elements and ego left intact on the server.")
        return 0
    finally:
        # Intentionally NOT destroying the ego or removing obstacles:
        # the scene must persist after the client exits.
        log.debug("Client exiting without teardown.")


if __name__ == "__main__":
    sys.exit(main())
