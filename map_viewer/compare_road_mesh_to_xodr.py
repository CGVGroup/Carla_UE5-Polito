#!/usr/bin/env python3
"""
compare_road_mesh_to_xodr.py — Compares the actual visual road mesh placed in
the level against the .xodr-derived waypoints, across the whole map, to find
where they've drifted apart (roads that exist visually with no matching
logical road data, or vice versa).

Uses world.get_environment_objects(carla.CityObjectLabel.Roads) for the real,
placed road mesh instances (independent of the .xodr), and
world.get_map().generate_waypoints() for the logical OpenDRIVE network - then
flags mesh instances with no nearby waypoint (visual road missing from the
.xodr) and, separately, waypoints with no nearby mesh (.xodr road with no
matching visual geometry, e.g. a road that was moved).

Usage:
    python compare_road_mesh_to_xodr.py
    python compare_road_mesh_to_xodr.py -o mismatch.png --threshold 4.0
"""

import argparse

import carla
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--timeout', type=float, default=20.0, help='Client connection timeout in seconds (default 20)')
    ap.add_argument('--resolution', type=float, default=3.0,
                     help='Waypoint sampling resolution in metres (default 3.0)')
    ap.add_argument('--threshold', type=float, default=4.0,
                     help='Distance in metres beyond which a mesh/waypoint counts as mismatched (default 4.0)')
    ap.add_argument('-o', '--output', default='road_mesh_vs_xodr.png', help='Output PNG path')
    ap.add_argument('--max-print', type=int, default=40, help='Max mismatch locations to print per category (default 40)')
    args = ap.parse_args()

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    cmap = world.get_map()
    print(f'Connected. Map: {cmap.name}')

    print('Fetching placed road mesh instances (CityObjectLabel.Roads) ...')
    road_objects = world.get_environment_objects(carla.CityObjectLabel.Roads)
    mesh_points = np.array([[o.transform.location.x, o.transform.location.y] for o in road_objects])
    print(f'{len(road_objects)} road mesh instances')

    print(f'Generating .xodr waypoints at {args.resolution} m resolution ...')
    waypoints = cmap.generate_waypoints(args.resolution)
    wp_points = np.array([[w.transform.location.x, w.transform.location.y] for w in waypoints])
    print(f'{len(waypoints)} waypoints')

    if len(mesh_points) == 0 or len(wp_points) == 0:
        print('ERROR: one of the two sets is empty, cannot compare. '
              '(No CityObjectLabel.Roads instances, or no waypoints generated.)')
        return

    def nearest_distances(points_a, points_b, chunk=500):
        """For each point in a, the distance to the nearest point in b (chunked to bound memory)."""
        out = np.empty(len(points_a))
        for i in range(0, len(points_a), chunk):
            chunk_a = points_a[i:i + chunk]
            d = np.sqrt(((chunk_a[:, None, :] - points_b[None, :, :]) ** 2).sum(axis=2))
            out[i:i + chunk] = d.min(axis=1)
        return out

    print('Computing mesh -> nearest waypoint distances ...')
    mesh_to_wp = nearest_distances(mesh_points, wp_points)
    print('Computing waypoint -> nearest mesh distances ...')
    wp_to_mesh = nearest_distances(wp_points, mesh_points)

    mesh_mismatch = mesh_to_wp > args.threshold
    wp_mismatch = wp_to_mesh > args.threshold

    print()
    print(f'Road mesh instances with no waypoint within {args.threshold} m '
          f'(visual road missing from the .xodr): {mesh_mismatch.sum()} / {len(mesh_points)}')
    print(f'Waypoints with no road mesh within {args.threshold} m '
          f'(.xodr road with no matching visual geometry): {wp_mismatch.sum()} / {len(wp_points)}')

    if mesh_mismatch.sum():
        print(f'\nSample mismatched mesh locations (up to {args.max_print}):')
        for (x, y), d in list(zip(mesh_points[mesh_mismatch], mesh_to_wp[mesh_mismatch]))[:args.max_print]:
            print(f'    ({x:.1f}, {y:.1f})  nearest waypoint {d:.1f} m away')

    if wp_mismatch.sum():
        print(f'\nSample mismatched waypoint locations (up to {args.max_print}):')
        for (x, y), d in list(zip(wp_points[wp_mismatch], wp_to_mesh[wp_mismatch]))[:args.max_print]:
            print(f'    ({x:.1f}, {y:.1f})  nearest road mesh {d:.1f} m away')

    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#0d0d0d')
    ax.set_facecolor('#0d0d0d')

    ax.scatter(wp_points[~wp_mismatch, 0], wp_points[~wp_mismatch, 1],
               s=2, color='#3dd870', label='Waypoint (.xodr), matched')
    ax.scatter(mesh_points[~mesh_mismatch, 0], mesh_points[~mesh_mismatch, 1],
               s=2, color='#4090ff', label='Road mesh, matched')
    if wp_mismatch.sum():
        ax.scatter(wp_points[wp_mismatch, 0], wp_points[wp_mismatch, 1],
                   s=14, color='#ffb020', label='Waypoint with no nearby mesh', zorder=5)
    if mesh_mismatch.sum():
        ax.scatter(mesh_points[mesh_mismatch, 0], mesh_points[mesh_mismatch, 1],
                   s=14, color='#ff4f4f', label='Road mesh with no nearby waypoint', zorder=5)

    ax.set_title(f'{cmap.name} - road mesh vs .xodr waypoints (threshold {args.threshold} m)',
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
