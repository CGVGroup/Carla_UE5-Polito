#!/usr/bin/env python3
"""
show_route_live.py — Draw a route's waypoints and scenario trigger points
directly in the running CARLA server, as debug markers in the 3D world.

This is the live, in-editor equivalent of route_map_viewer.py's 2D plot —
no leaderboard/.sh/.ps1 run needed, just the route XML and a live server.

Usage:
    python show_route_live.py -r routes_Town15_long3.xml
    python show_route_live.py -r routes_Town15_long3.xml --life-time 300
    python show_route_live.py -r routes_Town15_long3.xml --route-id 0 --no-yaw
"""

import argparse
import math
import xml.etree.ElementTree as ET

import carla

# Colour per scenario type, matching route_map_viewer.py's palette (RGB 0-255)
SCENARIO_COLORS = {
    'ConstructionObstacleTwoWays':   (255, 140, 0),
    'ConstructionObstacle':          (255, 140, 0),
    'NonSignalizedJunctionLeftTurn': (30, 144, 255),
    'SignalizedJunctionLeftTurn':    (30, 144, 255),
    'DynamicObjectCrossing':         (255, 48, 48),
    '_default':                      (204, 0, 204),
}
ROUTE_COLOR = (0, 255, 127)
LLM_COLOR = (255, 215, 0)
FAIL_COLOR = (255, 0, 0)


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
                    'x': float(pos.get('x', 0)), 'y': float(pos.get('y', 0)), 'z': float(pos.get('z', 0)),
                    'text': pos.get('text', ''),
                })

        fail_triggers = []
        fail_el = route_el.find('failtriggers')
        if fail_el is not None:
            for pos in fail_el.findall('position'):
                fail_triggers.append({
                    'x': float(pos.get('x', 0)), 'y': float(pos.get('y', 0)), 'z': float(pos.get('z', 0)),
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
                    'z':   float(tp.get('z', 0)),
                    'yaw': float(tp.get('yaw', 0)),
                })

        routes.append({
            'id': route_el.get('id', '0'), 'town': route_el.get('town', ''),
            'waypoints': waypoints, 'llm_triggers': llm_triggers,
            'fail_triggers': fail_triggers, 'scenarios': scenarios,
        })
    return routes


def draw_marker(world, location, color, label, life_time, size, yaw=None, arrow_len=4.0):
    world.debug.draw_point(location, size=size, color=color, life_time=life_time)
    if label:
        world.debug.draw_string(location + carla.Location(z=0.5), label, draw_shadow=True,
                                 color=color, life_time=life_time, persistent_lines=True)
    if yaw is not None:
        rad = math.radians(yaw)
        end = location + carla.Location(x=arrow_len * math.cos(rad), y=arrow_len * math.sin(rad))
        world.debug.draw_arrow(location, end, thickness=0.1, arrow_size=0.3,
                                color=color, life_time=life_time)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-r', '--route', required=True, help='Path to the routes XML file')
    ap.add_argument('--route-id', default=None, help='Route id to draw (default: first)')
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--timeout', type=float, default=10.0, help='Client connection timeout in seconds (default 10)')
    ap.add_argument('--life-time', type=float, default=120.0,
                     help='How long the markers stay drawn, in seconds (default 120; 0 = single frame only)')
    ap.add_argument('--z-offset', type=float, default=2.0,
                     help='Extra height above ground so markers are not hidden by the road mesh (default 2.0)')
    ap.add_argument('--point-size', type=float, default=0.2, help='Point size in metres (default 0.2)')
    ap.add_argument('--no-yaw', action='store_true', help='Hide yaw direction arrows')
    args = ap.parse_args()

    show_yaw = not args.no_yaw

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

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    print(f'Connected. Current map: {world.get_map().name}')

    color = carla.Color(*ROUTE_COLOR)
    wp = route['waypoints']
    prev_loc = None
    for i, p in enumerate(wp):
        loc = carla.Location(p['x'], p['y'], p['z'] + args.z_offset)
        draw_marker(world, loc, color, str(i), args.life_time, args.point_size,
                    yaw=p.get('yaw') if show_yaw and i == 0 else None)
        if prev_loc is not None:
            world.debug.draw_line(prev_loc, loc, thickness=0.08, color=color, life_time=args.life_time)
        prev_loc = loc

    for sc in route['scenarios']:
        rgb = SCENARIO_COLORS.get(sc['type'], SCENARIO_COLORS['_default'])
        sc_color = carla.Color(*rgb)
        loc = carla.Location(sc['x'], sc['y'], sc['z'] + args.z_offset + 1.0)
        draw_marker(world, loc, sc_color, f"{sc['name']}", args.life_time, args.point_size * 1.5,
                    yaw=sc['yaw'] if show_yaw else None)

    llm_color = carla.Color(*LLM_COLOR)
    for t in route['llm_triggers']:
        loc = carla.Location(t['x'], t['y'], t['z'] + args.z_offset)
        draw_marker(world, loc, llm_color, t.get('text', 'LLM trigger'), args.life_time, args.point_size)

    fail_color = carla.Color(*FAIL_COLOR)
    for t in route['fail_triggers']:
        loc = carla.Location(t['x'], t['y'], t['z'] + args.z_offset)
        draw_marker(world, loc, fail_color, 'fail trigger', args.life_time, args.point_size)

    print(f"Drew {len(wp)} route waypoints, {len(route['scenarios'])} scenario markers, "
          f"{len(route['llm_triggers'])} LLM triggers, {len(route['fail_triggers'])} fail triggers "
          f"(life_time={args.life_time}s).")
    print('Look at the CARLA viewport/spectator to inspect them in place.')


if __name__ == '__main__':
    main()
