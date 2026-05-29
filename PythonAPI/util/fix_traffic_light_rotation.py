#!/usr/bin/env python

# Rotates all traffic lights by a yaw offset to correct their facing direction.
# Default: -90 degrees (fixes lights that face right instead of forward).

import argparse
import carla


def main():
    argparser = argparse.ArgumentParser(
        description='Fix traffic light yaw rotation')
    argparser.add_argument(
        '--host', metavar='H', default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port', metavar='P', default=2000, type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--yaw-offset', default=-90.0, type=float,
        help='Yaw degrees to add to every traffic light (default: -90)')
    argparser.add_argument(
        '--dry-run', action='store_true',
        help='Print what would change without applying it')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    world = client.get_world()

    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    print(f'Found {len(traffic_lights)} traffic lights')

    for tl in traffic_lights:
        t = tl.get_transform()
        old_yaw = t.rotation.yaw
        new_yaw = old_yaw + args.yaw_offset
        print(f'  TL id={tl.id}  yaw {old_yaw:.1f} -> {new_yaw:.1f}')
        if not args.dry_run:
            t.rotation.yaw = new_yaw
            tl.set_transform(t)

    if args.dry_run:
        print('Dry run — no changes applied.')
    else:
        print('Done.')


if __name__ == '__main__':
    main()
