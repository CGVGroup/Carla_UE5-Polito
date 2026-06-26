#!/usr/bin/env python3
"""
ARHUD client 2/3 — obstacles.

Spawns one obstacle per type, each 2 m above the ground, orbiting a common
centre on its OWN circle radius. Each stays upright and rotates only about Z so
its front (+x) face keeps pointing along travel (tangent). Streams continuously
until Ctrl+C. update_obstacle_dashboard is keyed by ID (idempotent), so this
never needs clear and won't disturb a path/ego client running alongside.

Vector1 = Position (world, x scale).  Vector2 = Rotation = (0,0,yaw).
"""
import argparse, logging, math, sys, time
import carla

log = logging.getLogger("c2")
TYPES = ["car", "pedestrian", "bike", "cone", "sign", "truck", "barrier", "animal"]


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
    ap.add_argument("--height", type=float, default=2.0, help="metres above ground")
    ap.add_argument("--base-radius", type=float, default=8.0, help="innermost circle radius (m)")
    ap.add_argument("--radius-step", type=float, default=4.0, help="radius increment per type (m)")
    ap.add_argument("--omega", type=float, default=0.4, help="angular speed (rad/s)")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(message)s", datefmt="%H:%M:%S")

    client = carla.Client(args.host, args.port); client.set_timeout(args.timeout)
    world = client.get_world()
    log.info("Server %s | Client %s", client.get_server_version(), client.get_client_version())
    if not hasattr(world, "update_obstacle_dashboard"):
        log.error("World missing update_obstacle_dashboard — rebuild PythonAPI."); return 4

    cx, cy, cz = 0.0, 0.0, 0.0
    S, N = args.scale, len(TYPES)
    z = cz + args.height
    radii = [args.base_radius + i * args.radius_step for i in range(N)]
    outer = radii[-1]

    spec = world.get_spectator()
    spec.set_transform(carla.Transform(carla.Location(cx, cy, cz + outer * 2.2),
                                       carla.Rotation(pitch=-90)))
    call(world, "set_type_string_dashboard", "ARHUD")
    log.info("%d obstacles, radii %.0f..%.0f m, %.0f m up. Ctrl+C to detach.",
             N, radii[0], radii[-1], args.height)
    for i, t in enumerate(TYPES):
        log.info("  id=%d %-11s r=%.0fm", i + 1, t, radii[i])

    try:
        t0 = time.time(); last = t0
        while True:
            world.wait_for_tick()
            now = time.time()
            if now - last < args.step:
                time.sleep(0.005); continue
            last = now
            t = now - t0
            for i, typ in enumerate(TYPES):
                r = radii[i]
                ang = args.omega * t + i * (2.0 * math.pi / N)   # spread around
                x = cx + r * math.cos(ang)
                y = cy + r * math.sin(ang)
                yaw = math.degrees(ang) + 90.0                   # tangent (CCW): front forward
                call(world, "update_obstacle_dashboard",
                     i + 1, typ, bool(i % 2 == 0),
                     carla.Vector3D(x * S, y * S, z * S),         # position
                     carla.Vector3D(0.0, 0.0, yaw),               # z-only rotation
                     args.omega * r * 3.6, 0.0)
            call(world, "update_dashboard")
    except KeyboardInterrupt:
        log.info("Detaching — obstacles left in place."); return 0


if __name__ == "__main__":
    sys.exit(main())
