#!/usr/bin/env python3
"""
test_walker_control.py — Isolates whether raw carla.WalkerControl actually
moves a pedestrian on this server, completely outside the leaderboard/
scenario_runner machinery.

Spawns one walker, applies a constant WalkerControl(speed, direction) exactly
like KeepVelocity (srunner/scenariomanager/scenarioatomics/atomic_behaviors.py)
does, and prints its position/speed every tick for --duration seconds. If the
walker's reported speed never approaches --speed, walker movement is broken
at the server level (independent of any scenario_runner scenario), not a
scenario-specific bug.

Usage:
    python test_walker_control.py
    python test_walker_control.py --speed 2.0 --duration 8
"""

import argparse
import math
import time

import carla


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--timeout', type=float, default=10.0, help='Client connection timeout in seconds (default 10)')
    ap.add_argument('--speed', type=float, default=2.0, help='Target walker speed in m/s (default 2.0)')
    ap.add_argument('--duration', type=float, default=8.0, help='How long to drive the walker, in seconds (default 8)')
    ap.add_argument('--walker', default='walker.pedestrian.*', help='Walker blueprint filter')
    args = ap.parse_args()

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()

    bps = world.get_blueprint_library().filter(args.walker)
    if not bps:
        print(f'ERROR: no blueprint matching {args.walker!r}')
        all_walkers = world.get_blueprint_library().filter('walker.*')
        if all_walkers:
            print(f'Available walker.* blueprints in this content build ({len(all_walkers)}):')
            for bp in all_walkers:
                print(f'    {bp.id}')
        else:
            print('No walker.* blueprints exist in this content build at all.')
        return
    walker_bp = bps[0]
    print(f'Using blueprint: {walker_bp.id} (out of {len(bps)} matching)')

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print('ERROR: map has no spawn points')
        return

    walker = None
    for sp in spawn_points:
        walker = world.try_spawn_actor(walker_bp, sp)
        if walker:
            break
    if walker is None:
        print('ERROR: could not spawn walker at any spawn point (collisions?)')
        return

    print(f'Spawned walker id={walker.id} at {walker.get_location()}')
    world.wait_for_tick()

    start_location = walker.get_location()
    direction = carla.Vector3D(1.0, 0.0, 0.0)
    control = carla.WalkerControl(direction=direction, speed=args.speed)
    walker.apply_control(control)
    print(f'Applied WalkerControl(direction={direction}, speed={args.speed})')

    t0 = time.time()
    max_horiz_speed_seen = 0.0
    try:
        while time.time() - t0 < args.duration:
            world.wait_for_tick()
            vel = walker.get_velocity()
            # Horizontal (x/y) speed only - a vertical fall under gravity right after
            # spawn can otherwise look like a big "speed" that has nothing to do with
            # whether WalkerControl's direction/speed is actually being honored.
            horiz_speed = math.sqrt(vel.x ** 2 + vel.y ** 2)
            max_horiz_speed_seen = max(max_horiz_speed_seen, horiz_speed)
            loc = walker.get_location()
            horiz_traveled = math.sqrt((loc.x - start_location.x) ** 2 + (loc.y - start_location.y) ** 2)
            print(f't={time.time() - t0:5.1f}s  horiz_speed={horiz_speed:5.2f} m/s  '
                  f'horiz_traveled={horiz_traveled:5.2f} m  loc=({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})')
    finally:
        walker.destroy()

    print()
    print(f'Max horizontal speed observed: {max_horiz_speed_seen:.2f} m/s (target was {args.speed:.2f} m/s)')
    if max_horiz_speed_seen < args.speed * 0.5:
        print('RESULT: walker did NOT reach anywhere close to the target horizontal speed. '
              'Walker movement appears broken at the server level.')
    else:
        print('RESULT: walker moved horizontally as expected — WalkerControl works fine standalone.')


if __name__ == '__main__':
    main()
