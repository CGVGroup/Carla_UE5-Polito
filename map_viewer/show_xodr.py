#!/usr/bin/env python3
"""
show_xodr.py — Plot every waypoint of an OpenDRIVE (.xodr) map.

Parses the .xodr offline (no running CARLA server needed) via carla.Map(),
and scatter-plots all generated waypoints. Useful for checking whether a
.xodr's road network actually lines up with where you expect it to be
(e.g. comparing it against a route's waypoints, or against another .xodr).

Usage:
    python show_xodr.py Town15.xodr
    python show_xodr.py Town15.xodr --resolution 1.0 -o town15.png
    python show_xodr.py Town15.xodr --color-by road_id
"""

import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

# ── CARLA import ──────────────────────────────────────────────────────────────
try:
    sys.path.append(glob.glob(os.path.join(
        os.path.dirname(__file__), '../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major, sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass

import carla  # noqa: E402


def load_waypoints(xodr_path, resolution):
    with open(xodr_path, encoding='utf-8') as f:
        opendrive = f.read()
    cmap = carla.Map(os.path.basename(xodr_path), opendrive)
    return cmap.generate_waypoints(resolution)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('xodr', help='Path to the .xodr file')
    ap.add_argument('--resolution', type=float, default=2.0,
                     help='Waypoint sampling resolution in metres (default 2.0)')
    ap.add_argument('--color-by', choices=['none', 'road_id', 'lane_id'], default='road_id',
                     help='Colour waypoints by attribute, to help spot misaligned/overlapping roads (default road_id)')
    ap.add_argument('-o', '--output', default='', help='Output PNG path (omit to show an interactive window)')
    ap.add_argument('--dpi', type=int, default=150, help='Output DPI (default 150)')
    args = ap.parse_args()

    print(f'Loading {args.xodr} …')
    waypoints = load_waypoints(args.xodr, args.resolution)
    print(f'{len(waypoints)} waypoints at {args.resolution} m resolution')

    xs = np.array([w.transform.location.x for w in waypoints])
    ys = np.array([w.transform.location.y for w in waypoints])

    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#0d0d0d')
    ax.set_facecolor('#0d0d0d')

    if args.color_by == 'none':
        ax.scatter(xs, ys, s=2, color='#cccccc')
    else:
        key = (lambda w: w.road_id) if args.color_by == 'road_id' else (lambda w: w.lane_id)
        values = np.array([key(w) for w in waypoints])
        scatter = ax.scatter(xs, ys, s=2, c=values, cmap='tab20')
        cbar = fig.colorbar(scatter, ax=ax, fraction=0.03, pad=0.02)
        cbar.set_label(args.color_by, color='white')
        cbar.ax.yaxis.set_tick_params(color='white')
        plt.setp(plt.getp(cbar.ax, 'yticklabels'), color='white')

    ax.set_title(f'{os.path.basename(args.xodr)} — {len(waypoints)} waypoints @ {args.resolution} m',
                 color='white', fontsize=11, pad=10)
    ax.tick_params(colors='#666666', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')
    ax.set_xlabel('X (m)', color='#888888', fontsize=8)
    ax.set_ylabel('Y (m)', color='#888888', fontsize=8)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.grid(True, color='#222222', linewidth=0.5, linestyle='--')

    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=args.dpi, bbox_inches='tight', facecolor=fig.get_facecolor())
        print(f'Saved -> {args.output}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
