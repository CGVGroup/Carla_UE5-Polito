#!/usr/bin/env python3
"""
show_waypoints_live.py — Draw every waypoint of the map physically in the
running CARLA server, as debug points in the 3D world.

Unlike show_xodr.py (which parses a .xodr file offline), this connects to a
live CARLA server and asks ITS map for the waypoints — so what you see is
exactly what the server thinks the road network looks like, in its own
world coordinates. This is the right tool to catch a server-side alignment
issue between the road network and the rendered level (a .xodr that looks
fine in show_xodr.py can still be offset from the actual .umap geometry).

Usage:
    python show_waypoints_live.py
    python show_waypoints_live.py --host 127.0.0.1 --port 2000 --resolution 5.0
    python show_waypoints_live.py --life-time 300 --color-by lane_id
"""

import argparse
import sys

import carla


def color_for(value, palette):
    r, g, b = palette[value % len(palette)]
    return carla.Color(r, g, b)


# tab20-style palette (RGB 0-255), cycled through by road_id/lane_id
PALETTE = [
    (31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40),
    (148, 103, 189), (140, 86, 75), (227, 119, 194), (127, 127, 127),
    (188, 189, 34), (23, 190, 207), (174, 199, 232), (255, 187, 120),
    (152, 223, 138), (255, 152, 150), (197, 176, 213), (196, 156, 148),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--timeout', type=float, default=10.0, help='Client connection timeout in seconds (default 10)')
    ap.add_argument('--resolution', type=float, default=5.0,
                     help='Waypoint sampling resolution in metres (default 5.0; lower = more points = slower)')
    ap.add_argument('--life-time', type=float, default=60.0,
                     help='How long the points stay drawn, in seconds (default 60; 0 = single frame only)')
    ap.add_argument('--size', type=float, default=0.15, help='Point size in metres (default 0.15)')
    ap.add_argument('--z-offset', type=float, default=2.5,
                     help='Extra height above the road surface, so points are not hidden by the mesh (default 2.5)')
    ap.add_argument('--color-by', choices=['road_id', 'lane_id', 'none'], default='road_id',
                     help='Colour points by attribute (default road_id)')
    args = ap.parse_args()

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    cmap = world.get_map()
    print(f'Connected. Current map: {cmap.name}')

    print(f'Generating waypoints at {args.resolution} m resolution ...')
    waypoints = cmap.generate_waypoints(args.resolution)
    print(f'{len(waypoints)} waypoints. Drawing them in the world (life_time={args.life_time}s) ...')

    for i, wp in enumerate(waypoints):
        location = wp.transform.location
        location.z += args.z_offset

        if args.color_by == 'none':
            color = carla.Color(0, 255, 120)
        elif args.color_by == 'road_id':
            color = color_for(wp.road_id, PALETTE)
        else:
            color = color_for(wp.lane_id, PALETTE)

        world.debug.draw_point(location, size=args.size, color=color, life_time=args.life_time)

        if (i + 1) % 1000 == 0:
            print(f'  {i + 1}/{len(waypoints)} drawn ...')

    print('Done. Look at the CARLA viewport/spectator to inspect the waypoints in place.')
    if args.life_time > 0:
        print(f'They will disappear after {args.life_time} seconds; re-run to refresh.')


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as e:
        print(f'ERROR: could not reach the CARLA server ({e}). Is it running and reachable at that host/port?',
              file=sys.stderr)
        sys.exit(1)
