#!/usr/bin/env python3
"""
check_waypoint_ground_alignment.py — For every .xodr waypoint, casts a ray
straight down (world.ground_projection) to find the actual physical geometry
beneath it, and reports:
  - waypoints with NO ground hit at all within range (a void - no collision
    geometry there, i.e. the .xodr claims a road exists but there's nothing
    physically there)
  - waypoints whose vertical distance to the ground hit exceeds a threshold
    (the road network says the road surface is at height Z, but the actual
    ground there is at a notably different height - an elevation mismatch,
    which would make the waypoint visibly float above/below the real road)

Use this when compare_road_mesh_to_xodr.py comes up empty (no geometry
tagged CityObjectLabel.Roads) - this checks physical collision geometry
directly instead of relying on semantic tags.

Usage:
    python check_waypoint_ground_alignment.py
    python check_waypoint_ground_alignment.py --resolution 5.0 --z-threshold 2.0
"""

import argparse

import carla
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--timeout', type=float, default=20.0, help='Client connection timeout in seconds (default 20)')
    ap.add_argument('--resolution', type=float, default=8.0,
                     help='Waypoint sampling resolution in metres (default 8.0; lower = more RPC calls = slower)')
    ap.add_argument('--z-threshold', type=float, default=1.5,
                     help='Vertical mismatch in metres to flag as misaligned (default 1.5)')
    ap.add_argument('--search-distance', type=float, default=200.0,
                     help='Max downward raycast search distance in metres (default 200)')
    ap.add_argument('--max-print', type=int, default=60, help='Max locations to print per category (default 60)')
    ap.add_argument('-o', '--output', default='waypoint_ground_alignment.png', help='Output PNG path')
    args = ap.parse_args()

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    cmap = world.get_map()
    print(f'Connected. Map: {cmap.name}')

    waypoints = cmap.generate_waypoints(args.resolution)
    print(f'{len(waypoints)} waypoints at {args.resolution} m resolution; '
          f'querying ground projection for each (one RPC call per waypoint, this can take a while) ...')

    no_hit = []
    mismatched = []
    matched = []
    for i, wp in enumerate(waypoints):
        loc = wp.transform.location
        query_loc = carla.Location(loc.x, loc.y, loc.z + 5.0)
        hit = world.ground_projection(query_loc, args.search_distance)
        if hit is None:
            no_hit.append(wp)
        else:
            dz = abs(hit.location.z - loc.z)
            if dz > args.z_threshold:
                mismatched.append((wp, hit, dz))
            else:
                matched.append(wp)
        if (i + 1) % 1000 == 0:
            print(f'  {i + 1}/{len(waypoints)} ...')

    print()
    print(f'No ground hit at all (void - road claimed but nothing physically there): '
          f'{len(no_hit)} / {len(waypoints)}')
    print(f'Ground hit but z-mismatch > {args.z_threshold} m (elevation drift): '
          f'{len(mismatched)} / {len(waypoints)}')
    print(f'Matched (within tolerance): {len(matched)} / {len(waypoints)}')

    if no_hit:
        print(f'\nSample no-hit locations (up to {args.max_print}):')
        for wp in no_hit[:args.max_print]:
            l = wp.transform.location
            print(f'    ({l.x:.1f}, {l.y:.1f}, {l.z:.1f})')

    if mismatched:
        print(f'\nSample z-mismatch locations (up to {args.max_print}):')
        for wp, hit, dz in mismatched[:args.max_print]:
            l = wp.transform.location
            print(f'    wp=({l.x:.1f}, {l.y:.1f}, {l.z:.1f})  ground_hit_z={hit.location.z:.1f}  '
                  f'dz={dz:.1f} m  label={hit.label}')

    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#0d0d0d')
    ax.set_facecolor('#0d0d0d')

    def xy(wps):
        return [w.transform.location.x for w in wps], [w.transform.location.y for w in wps]

    if matched:
        mx, my = xy(matched)
        ax.scatter(mx, my, s=2, color='#3dd870', label='Matched')
    if mismatched:
        mmx = [m[0].transform.location.x for m in mismatched]
        mmy = [m[0].transform.location.y for m in mismatched]
        ax.scatter(mmx, mmy, s=12, color='#ffb020', label=f'Z-mismatch > {args.z_threshold} m', zorder=5)
    if no_hit:
        nx, ny = xy(no_hit)
        ax.scatter(nx, ny, s=12, color='#ff4f4f', label='No ground hit', zorder=6)

    ax.set_title(f'{cmap.name} - waypoint ground alignment (threshold {args.z_threshold} m)',
                 color='white', fontsize=11, pad=10)
    ax.tick_params(colors='#666666', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')
    ax.set_xlabel('X (m)', color='#888888', fontsize=8)
    ax.set_ylabel('Y (m)', color='#888888', fontsize=8)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.grid(True, color='#222222', linewidth=0.5, linestyle='--')
    ax.legend(loc='upper left', facecolor='#1a1a1a', edgecolor='#555555', labelcolor='white', fontsize=7)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f'\nSaved -> {args.output}')


if __name__ == '__main__':
    main()
