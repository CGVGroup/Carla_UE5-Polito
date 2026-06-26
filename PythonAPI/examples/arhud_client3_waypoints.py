#!/usr/bin/env python3
"""
ARHUD client 3/3 — figure-8 path.

Lays a static set of waypoints describing a figure-8 (two tangent circles,
radius --radius), 20 cm above the ground, then keeps the connection alive so the
path persists for study. add_vector_pair_dashboard only appends, so by default
this draws ONCE and does not clear (safe to run alongside the obstacle/ego
clients). Re-running duplicates unless you pass --clear (requires clear_dashboard
and will also wipe obstacles). Ctrl+C detaches; waypoints are left in place.

Each waypoint: Position (world, x scale) + Rotation (0,0,tangent_yaw).
"""
import argparse, logging, math, sys, time
import carla

log = logging.getLogger("c3")


def figure8(cx, cy, R, p):
    tp = p % (4.0 * math.pi)
    if tp < 2.0 * math.pi:
        return cx + R * math.sin(tp), cy + R * (1.0 - math.cos(tp))
    q = tp - 2.0 * math.pi
    return cx - R * math.sin(q), cy - R * (1.0 - math.cos(q))


def tangent_yaw(cx, cy, R, p):
    x0, y0 = figure8(cx, cy, R, p)
    x1, y1 = figure8(cx, cy, R, p + 1e-3)
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def call(world, m, *a):
    fn = getattr(world, m, None)
    if not fn:
        return
    try:
        fn(*a)
    except Exception as e:  # noqa: BLE001
        log.error("✗ %s: %s", m, e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--scale", type=float, default=100.0, help="m->units (100=cm)")
    ap.add_argument("--radius", type=float, default=30.0, help="figure-8 loop radius (m)")
    ap.add_argument("--height", type=float, default=0.2, help="metres above ground (0.2 = 20 cm)")
    ap.add_argument("--max-points", type=int, default=10, help="waypoints along the 8 (max)")
    ap.add_argument("--car", default="vehicle.mercedes.*", help="parked car blueprint")
    ap.add_argument("--car-z", type=float, default=0.0, help="spawn height for the car (m)")
    ap.add_argument("--no-clear", action="store_true", help="don't clear before drawing")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(message)s", datefmt="%H:%M:%S")

    client = carla.Client(args.host, args.port); client.set_timeout(args.timeout)
    world = client.get_world()
    log.info("Server %s | Client %s", client.get_server_version(), client.get_client_version())
    for need in ("add_vector_pair_dashboard", "trigger_path_dashboard", "update_dashboard"):
        if not hasattr(world, need):
            log.error("World missing %s — rebuild PythonAPI.", need); return 4

    cx, cy, cz = 0.0, 0.0, 0.0
    S, R = args.scale, args.radius
    z = cz + args.height
    M = max(2, args.max_points)
    dphi = (4.0 * math.pi) / M

    if not args.no_clear:
        if hasattr(world, "clear_dashboard"):
            call(world, "clear_dashboard")
        else:
            log.warning("clear_dashboard RPC absent — old waypoints may accumulate.")

    spec = world.get_spectator()
    spec.set_transform(carla.Transform(carla.Location(cx, cy + R, cz + R * 2.5),
                                       carla.Rotation(pitch=-75)))

    # park a still Mercedes at the centre (0,0,0)
    vehicle = None
    cbps = world.get_blueprint_library().filter(args.car)
    if not cbps:
        log.warning("Blueprint '%s' not found — no car spawned.", args.car)
    else:
        for zoff in (args.car_z, args.car_z + 0.5, args.car_z + 1.0):
            vehicle = world.try_spawn_actor(cbps[0], carla.Transform(carla.Location(0.0, 0.0, zoff)))
            if vehicle:
                break
        if vehicle:
            vehicle.set_autopilot(False)
            vehicle.apply_control(carla.VehicleControl(hand_brake=True))
            log.info("Parked %s id=%d at origin.", cbps[0].id, vehicle.id)
        else:
            log.warning("Could not spawn %s at origin (collision?).", args.car)

    for m in range(M):
        p = m * dphi
        wx, wy = figure8(cx, cy, R, p)
        call(world, "add_vector_pair_dashboard",
             carla.Vector3D(wx * S, wy * S, z * S),
             carla.Vector3D(0.0, 0.0, tangent_yaw(cx, cy, R, p)))
    call(world, "trigger_path_dashboard")
    call(world, "update_dashboard")
    log.info("Drew %d waypoints, figure-8 R=%.0fm at %.0f cm. Ctrl+C to detach.",
             M, R, args.height * 100.0)

    try:
        t = 0
        while True:
            world.wait_for_tick()
            time.sleep(0.2)
            call(world, "set_control_value_dashboard", 0.0, 0.0)   # parked -> populate/show HUD
            call(world, "update_dashboard")
            t += 1
            if t % 10 == 0:
                call(world, "trigger_path_dashboard")              # keep the path painted
    except KeyboardInterrupt:
        log.info("Detaching — waypoints and car left in place."); return 0


if __name__ == "__main__":
    sys.exit(main())
