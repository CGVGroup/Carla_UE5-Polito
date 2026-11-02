#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Finds the roundabouts of a map from its OpenDRIVE description.

OpenDRIVE doesn't mark roundabouts, so they are found geometrically: a roundabout is a short closed
loop of roads (following the driving direction) that turns a full circle. Returns the ids of all
the roads that are part of one, including the connecting roads of the junctions on its ring (these
join the ring with the roads going in and out of it).
"""

import json
import math
import os
import xml.etree.ElementTree as ET

import networkx as nx

MIN_LOOP_LENGTH = 40  # [m] Shorter loops are not roundabouts
MAX_LOOP_LENGTH = 210  # [m] Longer loops are not roundabouts (but ring roads around blocks)
FULL_CIRCLE_TOLERANCE = 0.9  # [rad] How far the total turning can be from a full circle
MIN_CURVED_FRACTION = 0.6  # Part of the loop that has to be curved (loops around blocks are mostly straight)
MIN_CURVATURE = 0.015  # [1/m] Curvature from which a road piece counts as curved (radius < ~67 m)
# A ring is a circle: its curved parts have the curvature of a circle of the loop's length (2 pi / length).
# Loops around blocks with rounded corners turn much tighter at their corners (~1.9 times, in Town15)
MAX_CURVATURE_RATIO = 1.3

_cache = {}


def _road_turn(road):
    """
    Total signed turning of a road along its reference line [rad], its curved length [m] and the total
    absolute turning of its curved parts [rad]
    """
    turn = 0.0
    curved = 0.0
    curved_turn = 0.0
    for geometry in road.find('planView').findall('geometry'):
        length = float(geometry.get('length'))
        shape = geometry[0]
        curvature = 0.0
        if shape.tag == 'arc':
            curvature = float(shape.get('curvature'))
        elif shape.tag == 'spiral':
            curvature = (float(shape.get('curvStart')) + float(shape.get('curvEnd'))) / 2
        turn += curvature * length
        if abs(curvature) > MIN_CURVATURE:
            curved += length
            curved_turn += abs(curvature) * length
    return turn, curved, curved_turn


def find_roundabout_roads_in_opendrive(opendrive):
    """Returns the sorted ids of the roads that are part of a roundabout, given the OpenDRIVE xml string"""
    root = ET.fromstring(opendrive.encode('utf-8') if isinstance(opendrive, str) else opendrive)
    roads = {int(road.get('id')): road for road in root.findall('road')}
    junctions = {int(junction.get('id')): junction for junction in root.findall('junction')}
    length = {road_id: float(road.get('length')) for road_id, road in roads.items()}
    turn, curved, curved_turn = {}, {}, {}
    for road_id, road in roads.items():
        turn[road_id], curved[road_id], curved_turn[road_id] = _road_turn(road)

    # Directed graph of road traversals. Node: (road id, 1 if driven from its start to its end, else -1)
    graph = nx.DiGraph()
    for road_id, road in roads.items():
        link = road.find('link')
        if link is None:
            continue
        for direction, end in ((1, 'successor'), (-1, 'predecessor')):
            element = link.find(end)
            if element is None:
                continue
            element_id = int(element.get('elementId'))
            if element.get('elementType') == 'road':
                contact = element.get('contactPoint')
                graph.add_edge((road_id, direction), (element_id, 1 if contact == 'start' else -1))
            elif element_id in junctions:
                for connection in junctions[element_id].findall('connection'):
                    if int(connection.get('incomingRoad')) == road_id:
                        connecting = int(connection.get('connectingRoad'))
                        contact = connection.get('contactPoint')
                        graph.add_edge((road_id, direction), (connecting, 1 if contact == 'start' else -1))

    # The ring of a roundabout is the shortest loop through any of its roads (whatever the amount of
    # small road pieces and junction connections it's made of), so check the shortest loop of every road
    for start, end in graph.edges:
        graph[start][end]['weight'] = length[end[0]]
    ring_roads = set()
    for node in graph.nodes:
        if node[0] in ring_roads:
            continue
        lengths, paths = nx.single_source_dijkstra(graph, node, cutoff=MAX_LOOP_LENGTH)
        best = None
        for previous in graph.predecessors(node):
            if previous in lengths:
                total_length = lengths[previous] + length[node[0]]
                if best is None or total_length < best[0]:
                    best = (total_length, paths[previous])
        if best is None or not MIN_LOOP_LENGTH < best[0] < MAX_LOOP_LENGTH:
            continue
        cycle = best[1]
        total_turn = sum(turn[n[0]] * n[1] for n in cycle)
        if abs(abs(total_turn) - 2 * math.pi) > FULL_CIRCLE_TOLERANCE:
            continue
        loop_curved = sum(curved[n[0]] for n in cycle)
        if loop_curved < MIN_CURVED_FRACTION * best[0]:
            continue  # a loop around a block (straight sides and a few corners), not a ring
        mean_curvature = sum(curved_turn[n[0]] for n in cycle) / loop_curved
        if mean_curvature > MAX_CURVATURE_RATIO * 2 * math.pi / best[0]:
            continue  # a loop around a block with rounded corners, not a ring
        ring_roads.update(n[0] for n in cycle)

    # The ring's junctions also contain the connecting roads that join the ring with the roads going in and out of it
    result = set(ring_roads)
    for junction in junctions.values():
        connecting_roads = [int(c.get('connectingRoad')) for c in junction.findall('connection')]
        if any(road_id in ring_roads for road_id in connecting_roads):
            result.update(connecting_roads)
    return sorted(result)


def get_roundabout_road_ids(carla_map, cache_dir=None):
    """
    Returns the set of road ids (as in carla.Waypoint.road_id) that are part of a roundabout of the map.
    The result is cached in memory, and on disk if a cache directory is given.
    """
    name = carla_map.name.split('/')[-1]
    if name in _cache:
        return _cache[name]

    cache_file = os.path.join(cache_dir, 'roundabouts_{}.json'.format(name)) if cache_dir else None
    road_ids = None
    if cache_file and os.path.isfile(cache_file):
        try:
            with open(cache_file) as f:
                road_ids = json.load(f)
        except (OSError, ValueError):
            road_ids = None

    if road_ids is None:
        road_ids = find_roundabout_roads_in_opendrive(carla_map.to_opendrive())
        if cache_file:
            try:
                with open(cache_file, 'w') as f:
                    json.dump(road_ids, f)
            except OSError:
                pass

    _cache[name] = set(road_ids)
    return _cache[name]
