#!/usr/bin/env python3
"""
Route map viewer — renders a CARLA routes XML on top of its OpenDRIVE map.

Usage:
    python route_map_viewer.py -r <route.xml> -f <Town.xodr> [-o output.png] [--dpi 150]
    python route_map_viewer.py -r <route.xml> -f <Town.xodr> --no-yaw   # hide direction arrows

Output: a PNG (or interactive window if -o is omitted) showing:
  • Road network (gray dots)
  • Route waypoints connected in order (green line + numbered dots)
  • Each scenario type in a distinct colour, optionally with a yaw arrow
  • LLM triggers and fail triggers
  • Legend
"""

import argparse
import glob
import math
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np

# ── CARLA import ──────────────────────────────────────────────────────────────
try:
    sys.path.append(glob.glob(os.path.join(
        os.path.dirname(__file__), '../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major, sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64')))[0])
except IndexError:
    pass

_EGG = '/home/tda/Desktop/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg'
if _EGG not in sys.path:
    sys.path.append(_EGG)

import carla  # noqa: E402

# ── Colours & markers per scenario type ──────────────────────────────────────
SCENARIO_STYLE = {
    'ConstructionObstacleTwoWays':   dict(color='#FF8C00', marker='s', label='ConstructionObstacleTwoWays', zorder=6),
    'ConstructionObstacle':          dict(color='#FF8C00', marker='s', label='ConstructionObstacle',        zorder=6),
    'NonSignalizedJunctionLeftTurn': dict(color='#1E90FF', marker='^', label='NonSignalizedJunctionLeftTurn', zorder=6),
    'SignalizedJunctionLeftTurn':    dict(color='#1E90FF', marker='^', label='SignalizedJunctionLeftTurn',    zorder=6),
    'DynamicObjectCrossing':         dict(color='#FF3030', marker='o', label='DynamicObjectCrossing',        zorder=6),
    '_default':                      dict(color='#CC00CC', marker='D', label='Other scenario',               zorder=6),
}

LLM_STYLE   = dict(color='#FFD700', marker='*', s=120, zorder=7)
FAIL_STYLE  = dict(color='#FF0000', marker='x', s=120, zorder=7)
ROUTE_STYLE = dict(color='#00FF7F', marker='o', s=40,  zorder=5)
ROAD_STYLE  = dict(color='#404040', s=1,         zorder=1)


# ── XML parsing ───────────────────────────────────────────────────────────────

def parse_route_xml(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    routes = []
    for route_el in root.iter('route'):
        waypoints = []
        for pos in route_el.find('waypoints').findall('position'):
            waypoints.append({
                'x':   float(pos.get('x', 0)),
                'y':   float(pos.get('y', 0)),
                'z':   float(pos.get('z', 0)),
                'yaw': float(pos.get('yaw', 0)),
            })

        llm_triggers = []
        llm_el = route_el.find('llmtriggers')
        if llm_el is not None:
            for pos in llm_el.findall('position'):
                llm_triggers.append({
                    'x': float(pos.get('x', 0)),
                    'y': float(pos.get('y', 0)),
                    'text': pos.get('text', ''),
                })

        fail_triggers = []
        fail_el = route_el.find('failtriggers')
        if fail_el is not None:
            for pos in fail_el.findall('position'):
                fail_triggers.append({
                    'x': float(pos.get('x', 0)),
                    'y': float(pos.get('y', 0)),
                })

        scenarios = []
        sc_el = route_el.find('scenarios')
        if sc_el is not None:
            for sc in sc_el.findall('scenario'):
                tp = sc.find('trigger_point')
                if tp is None:
                    continue
                scenarios.append({
                    'name': sc.get('name', ''),
                    'type': sc.get('type', '_default'),
                    'x':   float(tp.get('x', 0)),
                    'y':   float(tp.get('y', 0)),
                    'yaw': float(tp.get('yaw', 0)),
                })

        routes.append({
            'id':           route_el.get('id', '0'),
            'town':         route_el.get('town', ''),
            'waypoints':    waypoints,
            'llm_triggers': llm_triggers,
            'fail_triggers':fail_triggers,
            'scenarios':    scenarios,
        })
    return routes


# ── Map loading ───────────────────────────────────────────────────────────────

def load_road_points(xodr_path, wp_resolution=4.0):
    with open(xodr_path) as f:
        opendrive = f.read()
    cmap = carla.Map('RouteViewer', opendrive)
    waypoints = cmap.generate_waypoints(wp_resolution)
    xs = np.array([w.transform.location.x for w in waypoints])
    ys = np.array([w.transform.location.y for w in waypoints])
    return xs, ys


# ── Drawing helpers ───────────────────────────────────────────────────────────

def draw_yaw_arrow(ax, x, y, yaw_deg, length, color, zorder=8):
    rad = math.radians(yaw_deg)
    dx  = length * math.cos(rad)
    dy  = length * math.sin(rad)
    ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5),
                zorder=zorder)


def plot_route(ax, route, arrow_len, show_yaw):
    wp = route['waypoints']
    xs = [p['x'] for p in wp]
    ys = [p['y'] for p in wp]

    ax.plot(xs, ys, color='#00FF7F', lw=1.2, zorder=4, alpha=0.7)
    ax.scatter(xs, ys, **{k: v for k, v in ROUTE_STYLE.items() if k != 'zorder'},
               zorder=ROUTE_STYLE['zorder'])

    for i, (x, y) in enumerate(zip(xs, ys)):
        ax.text(x, y + arrow_len * 0.6, str(i),
                fontsize=5, color='#00FF7F', ha='center', va='bottom', zorder=9,
                path_effects=[pe.withStroke(linewidth=1.5, foreground='black')])

    if show_yaw and len(wp) >= 2:
        draw_yaw_arrow(ax, xs[0], ys[0], wp[0].get('yaw', 0), arrow_len * 2, '#00FF7F')


def plot_scenarios(ax, scenarios, arrow_len, show_yaw, added_types):
    for sc in scenarios:
        style = SCENARIO_STYLE.get(sc['type'], SCENARIO_STYLE['_default'])
        x, y, yaw = sc['x'], sc['y'], sc['yaw']

        ax.scatter([x], [y],
                   c=style['color'], marker=style['marker'],
                   s=80, zorder=style['zorder'],
                   edgecolors='white', linewidths=0.5)

        if show_yaw:
            draw_yaw_arrow(ax, x, y, yaw, arrow_len * 1.5, style['color'])

        num = ''.join(c for c in sc['name'] if c.isdigit())
        ax.text(x, y - arrow_len * 0.8, num,
                fontsize=5.5, color=style['color'], ha='center', va='top', zorder=9,
                path_effects=[pe.withStroke(linewidth=1.5, foreground='black')])

        added_types.add((sc['type'], style['color'], style['marker']))


def plot_llm_triggers(ax, triggers):
    for t in triggers:
        ax.scatter([t['x']], [t['y']], **LLM_STYLE)
        if t.get('text'):
            ax.text(t['x'], t['y'], t['text'][:20],
                    fontsize=4, color='#FFD700', ha='left', va='bottom', zorder=10,
                    path_effects=[pe.withStroke(linewidth=1, foreground='black')])


def plot_fail_triggers(ax, triggers):
    for t in triggers:
        ax.scatter([t['x']], [t['y']], **FAIL_STYLE)


# ── Legend ────────────────────────────────────────────────────────────────────

def build_legend(ax, added_types, has_llm, has_fail, show_yaw):
    handles = [
        mpatches.Patch(color='#404040', label='Road network'),
        plt.Line2D([0], [0], color='#00FF7F', marker='o',
                   markersize=5, label='Route waypoints', linewidth=1.2),
    ]

    seen_labels = set()
    for sc_type, color, marker in sorted(added_types):
        style = SCENARIO_STYLE.get(sc_type, SCENARIO_STYLE['_default'])
        label = style['label']
        if label not in seen_labels:
            handles.append(plt.Line2D([0], [0], color=color, marker=marker,
                                       markersize=7, linestyle='None',
                                       markeredgecolor='white', markeredgewidth=0.5,
                                       label=label))
            seen_labels.add(label)

    if has_llm:
        handles.append(plt.Line2D([0], [0], color=LLM_STYLE['color'],
                                   marker=LLM_STYLE['marker'], markersize=8,
                                   linestyle='None', label='LLM trigger'))
    if has_fail:
        handles.append(plt.Line2D([0], [0], color=FAIL_STYLE['color'],
                                   marker=FAIL_STYLE['marker'], markersize=8,
                                   linestyle='None', label='Fail trigger'))
    if show_yaw:
        handles.append(plt.Line2D([0], [0], color='white', marker='>',
                                   markersize=6, linestyle='None',
                                   label='Trigger yaw direction'))

    ax.legend(handles=handles, loc='upper left',
              facecolor='#1a1a1a', edgecolor='#555555',
              labelcolor='white', fontsize=7, framealpha=0.85)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description='Render a CARLA route XML on its OpenDRIVE map')
    ap.add_argument('-r', '--route',    required=True, help='Path to the routes XML file')
    ap.add_argument('-f', '--file',     required=True, help='Path to the OpenDRIVE .xodr file')
    ap.add_argument('-o', '--output',   default='',    help='Output PNG path (omit to show window)')
    ap.add_argument('--dpi',            type=int, default=150, help='Output DPI (default 150)')
    ap.add_argument('--route-id',       default=None,  help='Route id to render (default: first)')
    ap.add_argument('--wp-resolution',  type=float, default=4.0,
                    help='Waypoint sampling resolution in metres (default 4)')
    ap.add_argument('--no-yaw',         action='store_true',
                    help='Hide yaw direction arrows on scenario trigger points')
    args = ap.parse_args()

    show_yaw = not args.no_yaw

    print('Loading road network…')
    road_xs, road_ys = load_road_points(args.file, args.wp_resolution)

    print('Parsing route XML…')
    routes = parse_route_xml(args.route)
    if not routes:
        print('No routes found in XML.')
        return

    route = routes[0]
    if args.route_id is not None:
        for r in routes:
            if r['id'] == args.route_id:
                route = r
                break

    x_range   = road_xs.max() - road_xs.min()
    y_range   = road_ys.max() - road_ys.min()
    arrow_len = max(x_range, y_range) * 0.012

    fig, ax = plt.subplots(figsize=(14, 14), facecolor='#0d0d0d')
    ax.set_facecolor('#0d0d0d')

    ax.scatter(road_xs, road_ys, **ROAD_STYLE)

    plot_route(ax, route, arrow_len, show_yaw)

    added_types = set()
    plot_scenarios(ax, route['scenarios'], arrow_len, show_yaw, added_types)

    has_llm  = bool(route['llm_triggers'])
    has_fail = bool(route['fail_triggers'])
    if has_llm:
        plot_llm_triggers(ax, route['llm_triggers'])
    if has_fail:
        plot_fail_triggers(ax, route['fail_triggers'])

    title = (f"Route {route['id']} — {route['town']}\n"
             f"{len(route['waypoints'])} waypoints · {len(route['scenarios'])} scenarios"
             + ('' if show_yaw else '  [yaw hidden]'))
    ax.set_title(title, color='white', fontsize=11, pad=10)
    ax.tick_params(colors='#666666', labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#333333')
    ax.set_xlabel('X (m)', color='#888888', fontsize=8)
    ax.set_ylabel('Y (m)', color='#888888', fontsize=8)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    ax.grid(True, color='#222222', linewidth=0.5, linestyle='--')

    build_legend(ax, added_types, has_llm, has_fail, show_yaw)

    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=args.dpi, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        print(f'Saved → {args.output}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
