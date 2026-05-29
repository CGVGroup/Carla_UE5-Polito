#!/usr/bin/env python
"""OpenDRIVE Map viewer
"""
# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import glob
import os
import sys
import argparse
import math
import time

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import pygame

# ==============================================================================
# -- Constants -----------------------------------------------------------------
# ==============================================================================

COLOR_BLACK = pygame.Color(0, 0, 0)
COLOR_GREEN = pygame.Color(0, 255, 0)
COLOR_RED = pygame.Color(255, 0, 0)
COLOR_BLUE = pygame.Color(0, 0, 255)
COLOR_YELLOW = pygame.Color(255, 255, 0)
COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_PINK = pygame.Color(255, 0, 255)


def world_to_pixel(location, pixels_per_meter, scale, world_offset, offset=(0, 0)):
    """Converts the world coordinates to pixel coordinates"""
    pixel_x = scale * pixels_per_meter * (location.x - world_offset[0])
    pixel_y = scale * pixels_per_meter * (location.y - world_offset[1])
    return [int(pixel_x - offset[0]), int(pixel_y - offset[1])]


def pixel_to_world(pixel_x, pixel_y, pixels_per_meter, scale, world_offset, offset=(0, 0)):
    """Converts the pixel coordinates to world coordinates"""
    location_x = float(pixel_x + offset[0]) / (scale * pixels_per_meter) + world_offset[0]
    location_y = float(pixel_y + offset[1]) / (scale * pixels_per_meter) + world_offset[1]
    return carla.Location(location_x, location_y, 0)


index = 0
capture_route_file = None
points_to_draw = []
marked_route = []
include_yaw = False   # set from --yaw flag in main()

def create_route_file():
    global index
    global capture_route_file

    try:
        capture_route_file = open(f"route_{index}.txt", 'w', encoding="utf-8")
        index += 1
        return True
    except OSError as error:
        print(error)
        return False


def close_route_file():
    global capture_route_file
    print("closing file")
    try:
        capture_route_file.close()
        capture_route_file =None
    except OSError as error:
        print(error)


def is_capturing_route():
    global capture_route_file
    return capture_route_file is not None

def start_route_capturing():
    global capture_route_file

    if is_capturing_route():
        return False
    
    if create_route_file():
        capture_route_file.write("<waypoints>\n")
    
    return is_capturing_route()


def add_route_point(location):
    # Store world location so pixel position can be recomputed after pan/zoom
    marked_route.append([location.location, location.rotation])
    points_to_draw.append(location.location)


def flush_route_to_file():
    global capture_route_file
    global marked_route
    global points_to_draw

    print("flushing")
    for point in marked_route:
        yaw_attr = f' yaw="{point[1].yaw:.2f}"' if include_yaw else ''
        capture_route_file.write(f'\t<position x="{point[0].x:.2f}" y="{point[0].y:.2f}" z="{point[0].z:.2f}"{yaw_attr}/>\n')
    
    marked_route = []
    points_to_draw = []

def stop_route_capturing():
    if is_capturing_route():
        flush_route_to_file()
        capture_route_file.write("</waypoints>\n")
        close_route_file()


# ==============================================================================
# -- Main --------------------------------------------------------------------
# ==============================================================================

HEIGHT = 1540
WIDTH = 810

def main():
    """Runs the 2D map viewer. Shows the map and the closest point of the mouse to the road
       Prints the required time to build the map structure and the average time of the query
       nearest point to the road.
    """

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '-f', '--file',
        metavar='F',
        default="",
        type=str,
        help='Path to the OpenDRIVE file')
    argparser.add_argument(
        '-o', '--osm',
        metavar='S',
        default="",
        type=str,
        help='Path to the OSM file')
    argparser.add_argument(
        '-c', '--center',
        action='store_true',
        help='Center the OSM map')
    argparser.add_argument(
        '-l', '--lights',
        action='store_true',
        help='Show traffic lights')
    argparser.add_argument(
        '-y', '--yaw',
        action='store_true',
        help='Include yaw attribute in output positions (default: omit yaw)')
    args = argparser.parse_args()

    global include_yaw
    include_yaw = args.yaw

    use_odr = False
    use_osm = False
    if (len(args.file) > 0):
        use_odr = True
    if (len(args.osm) > 0):
        use_osm = True
    if not use_odr and not use_osm:
        print("Error: no files specified")
        return
    if use_odr and use_osm:
        print("Error: use OpenDRIVE or OSM, nor both")
        return

    pygame.init()

    display = pygame.display.set_mode(
        (WIDTH, HEIGHT),
        pygame.HWSURFACE | pygame.DOUBLEBUF)

    # Place a title to game window
    pygame.display.set_caption("map viewer")

    opendrive = ""
    if use_odr:
        filename = args.file
        f_odr = open(filename, "r")
        opendrive = f_odr.read()
        f_odr.close()

    if use_osm:
        filename = args.osm
        f_odr = open(filename, "r")
        osm_data = f_odr.read()
        f_odr.close()
        settings = carla.Osm2OdrSettings()
        settings.center_map = True
        settings.proj_string = "+proj=tmerc"
        settings.generate_traffic_lights = True
        settings.all_junctions_with_traffic_lights = True
        settings.set_traffic_light_excluded_way_types([])
        #settings.set_osm_way_types(["motorway", "residential"])
        opendrive = carla.Osm2Odr.convert(osm_data, settings)
        f_odr = open("converted.xodr", "w")
        f_odr.write(opendrive)
        f_odr.close()


    start_map = time.time()
    carla_map = carla.Map("MapViewer", str(opendrive))
    end_map = time.time()
    print("Map load time: " + str(end_map - start_map) + " s")

    waypoints = carla_map.generate_waypoints(2)
    points = []
    x_list = []
    y_list = []
    for waypoint in waypoints:
        transf = waypoint.transform
        if math.isnan(transf.location.x) | math.isnan(transf.location.y):
            print("nan here: lane id " + str(waypoint.lane_id) +
                  " road id " + str(waypoint.road_id))
        else:
            x_list.append(transf.location.x)
            y_list.append(transf.location.y)
            points.append(transf)
    x_min = min(x_list)
    x_max = max(x_list)
    y_min = min(y_list)
    y_max = max(y_list)
    print("width " + str(x_max - x_min) + " height " + str(y_max - y_min))

    road_width = x_max - x_min
    road_height = y_max - y_min
    road_mid = (0.5 * (x_max + x_min), 0.5 * (y_max + y_min))
    base_scale = 0.99
    pixels_per_meter = min(WIDTH / max(1.0, road_width), HEIGHT / max(1.0, road_height))
    world_offset = road_mid

    # ── View state (zoom + pan) ──────────────────────────────────────────────
    scale = base_scale          # current zoom multiplier
    # pan_offset: pixel shift from the default centred view (positive = shift right/down)
    pan_offset = [0.0, 0.0]
    ZOOM_STEP = 1.15            # multiply/divide scale by this per wheel tick
    ZOOM_MIN  = base_scale * 0.5
    ZOOM_MAX  = base_scale * 50.0

    def view_offset():
        """Return the offset tuple expected by world_to_pixel / pixel_to_world."""
        return (-WIDTH / 2 + pan_offset[0], -HEIGHT / 2 + pan_offset[1])

    # Store road waypoint colours so we can redraw after zoom/pan
    road_point_colours = []
    for waypoint in waypoints:
        point = waypoint.transform
        color = COLOR_GREEN
        if waypoint.lane_id < 0:
            color = COLOR_YELLOW
        if waypoint.is_junction:
            color = COLOR_BLUE if waypoint.lane_id < 0 else COLOR_PINK
        road_point_colours.append((point.location, color))

    # Collect landmark data once (if requested)
    landmark_data = []
    if args.lights:
        for landmark in carla_map.get_all_landmarks_of_type("1000001"):
            wp = landmark.transform.location
            fwd = wp + 2 * landmark.transform.get_forward_vector()
            landmark_data.append((wp, fwd))
            print("landmark " + str(landmark.id) + " at " + str(wp))

    def render_road():
        """Redraw the road network onto `display` at the current scale/pan."""
        display.fill(COLOR_BLACK)
        vo = view_offset()
        for loc, color in road_point_colours:
            ps = world_to_pixel(loc, pixels_per_meter, scale, world_offset, vo)
            if 0 <= ps[0] < WIDTH and 0 <= ps[1] < HEIGHT:
                pygame.draw.circle(display, color, ps, 1)
        for wp_loc, fwd_loc in landmark_data:
            sp  = world_to_pixel(wp_loc,  pixels_per_meter, scale, world_offset, vo)
            fsp = world_to_pixel(fwd_loc, pixels_per_meter, scale, world_offset, vo)
            pygame.draw.circle(display, COLOR_WHITE, sp, 1)
            pygame.draw.line(display, COLOR_GREEN, sp, fsp)

    render_road()
    pygame.display.flip()
    road = display.convert()

    pygame.image.save(road, "map_viewer_image.png")
    print(str(len(waypoints)) + ' waypoints')
    waypoints = None

    road_dirty = False   # set True when we need to re-render the road layer

    landmark_distance = 10


    # waypoint = carla_map.get_waypoint(position)     # Get a waypoint
    # landmarks = waypoint.get_landmarks(50, False)   # Search for landmarks in 50m
    # for landmark in landmarks:
    #     if landmark.type == LandmarkType.StopSign:  # Find any landmark corresponding to a Stop
    #         distance_to_sign = landmark.distance    # Get distance from waypoint to the landmark in road m
    #         # Do stuff

    #points_to_draw = []

    

    signals_position = []
    signal_waypoint_position = []
    lines_to_draw = []
    font = pygame.font.SysFont("monospace", 14)
    panning = False          # middle-mouse drag state
    pan_start_mouse = (0, 0)
    pan_start_offset = [0.0, 0.0]

    while True:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            break

        # ── Zoom: mouse wheel ────────────────────────────────────────────────
        scroll = 0
        if event.type == pygame.MOUSEBUTTONDOWN and event.button in (4, 5):
            scroll = 1 if event.button == 4 else -1
        elif event.type == pygame.MOUSEWHEEL:
            scroll = event.y

        if scroll != 0:
            mx, my = pygame.mouse.get_pos()
            # world point under mouse before zoom
            vo = view_offset()
            mw = pixel_to_world(mx, my, pixels_per_meter, scale, world_offset, vo)
            # apply zoom
            factor = ZOOM_STEP if scroll > 0 else 1.0 / ZOOM_STEP
            new_scale = max(ZOOM_MIN, min(ZOOM_MAX, scale * factor))
            # adjust pan so the point under the mouse stays fixed
            pan_offset[0] = new_scale * pixels_per_meter * (mw.x - world_offset[0]) - mx + WIDTH / 2
            pan_offset[1] = new_scale * pixels_per_meter * (mw.y - world_offset[1]) - my + HEIGHT / 2
            scale = new_scale
            road_dirty = True

        # ── Keyboard zoom / pan / reset ──────────────────────────────────────
        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                mx, my = WIDTH // 2, HEIGHT // 2
                vo = view_offset()
                mw = pixel_to_world(mx, my, pixels_per_meter, scale, world_offset, vo)
                scale = min(ZOOM_MAX, scale * ZOOM_STEP)
                pan_offset[0] = scale * pixels_per_meter * (mw.x - world_offset[0]) - mx + WIDTH / 2
                pan_offset[1] = scale * pixels_per_meter * (mw.y - world_offset[1]) - my + HEIGHT / 2
                road_dirty = True
            elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                mx, my = WIDTH // 2, HEIGHT // 2
                vo = view_offset()
                mw = pixel_to_world(mx, my, pixels_per_meter, scale, world_offset, vo)
                scale = max(ZOOM_MIN, scale / ZOOM_STEP)
                pan_offset[0] = scale * pixels_per_meter * (mw.x - world_offset[0]) - mx + WIDTH / 2
                pan_offset[1] = scale * pixels_per_meter * (mw.y - world_offset[1]) - my + HEIGHT / 2
                road_dirty = True
            elif event.key in (pygame.K_0, pygame.K_r):
                scale = base_scale
                pan_offset[0] = 0.0
                pan_offset[1] = 0.0
                road_dirty = True
            elif event.key == pygame.K_LEFT:
                pan_offset[0] -= 40; road_dirty = True
            elif event.key == pygame.K_RIGHT:
                pan_offset[0] += 40; road_dirty = True
            elif event.key == pygame.K_UP:
                pan_offset[1] -= 40; road_dirty = True
            elif event.key == pygame.K_DOWN:
                pan_offset[1] += 40; road_dirty = True

        # ── Pan: middle-mouse drag ────────────────────────────────────────────
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
            panning = True
            pan_start_mouse = pygame.mouse.get_pos()
            pan_start_offset = pan_offset[:]
        if event.type == pygame.MOUSEBUTTONUP and event.button == 2:
            panning = False
        if panning and event.type == pygame.MOUSEMOTION:
            mx, my = pygame.mouse.get_pos()
            pan_offset[0] = pan_start_offset[0] + (mx - pan_start_mouse[0])
            pan_offset[1] = pan_start_offset[1] + (my - pan_start_mouse[1])
            road_dirty = True

        # ── Re-render road layer when view changed ───────────────────────────
        if road_dirty:
            render_road()
            road = display.convert()
            road_dirty = False

        display.blit(road, (0, 0))

        mouse = pygame.mouse.get_pos()
        mouse_position = pixel_to_world(mouse[0], mouse[1], pixels_per_meter,
                                        scale, world_offset, view_offset())
        mouse_waypoint = carla_map.get_waypoint(mouse_position)
        if mouse_waypoint is None:
            pygame.display.flip()
            continue

        if event.type == pygame.MOUSEBUTTONDOWN and event.button not in (2, 4, 5):

            if event.button == 3:
                stop_route_capturing()
                continue

            start_route_capturing()

            print("road id: " + str(mouse_waypoint.road_id) + " lane id: " + str(mouse_waypoint.lane_id))
            # ------------ROUTES TEST----------------------
            # message = "road: " + str(mouse_waypoint.road_id) + " lane: " + str(mouse_waypoint.lane_id)
            # print(message)
            # map_center_location = carla.Location(road_mid[0], road_mid[1], 0)
            # time_start = time.clock()
            # route = carla_map.compute_route(mouse_waypoint.transform.location, map_center_location)
            # time_end = time.clock()
            # print("query time: " + str(time_end - time_start))
            # print("number segments: " + str(route.get_number_segments()))
            # points_to_draw = []
            # center_waypoint = carla_map.get_waypoint(map_center_location)
            # points_to_draw.append(world_to_pixel(center_waypoint.transform.location,
            #                                     pixels_per_meter, scale, world_offset,
            #                                     (-WIDTH / 2, -HEIGHT / 2)))
            # points_to_draw.append(world_to_pixel(mouse_waypoint.transform.location,
            #                                     pixels_per_meter, scale, world_offset,
            #                                     (-WIDTH / 2, -HEIGHT / 2)))
            # route_waypoints = route.generate_waypoints(2)
            # signals_position = []
            # for waypoint in route_waypoints:
            #     signals_position.append(world_to_pixel(waypoint.transform.location,
            #                                     pixels_per_meter, scale, world_offset,
            #                                     (-WIDTH / 2, -HEIGHT / 2)))
            # ------------JUNCTION PRIORITY TEST-----------------------
            # if mouse_waypoint.is_junction:
            #     junction = mouse_waypoint.get_junction()
            #     time_start = time.clock()
            #     higher_priorities = junction.get_higher_priorities(mouse_waypoint.road_id)
            #     lower_priorities = junction.get_lower_priorities(mouse_waypoint.road_id)
            #     time_end = time.clock()
            #     print("query time: " + str(time_end - time_start))
            #     message = "Junction id: " + str(junction.id) + " Road: " + str(mouse_waypoint.road_id) + " High: "
            #     for high in higher_priorities:
            #         message += str(high) + " "
            #     message += " Low: "
            #     for low in lower_priorities:
            #         message += str(low) + " "
            #     print(message)

            # -------------LANDMARK TEST----------------
            #points_to_draw = []


            """
            waypoint_position = world_to_pixel(mouse_waypoint.transform.location,
                                           pixels_per_meter, scale, world_offset,
                                           (-WIDTH / 2, -HEIGHT / 2))
            
            marked_route.append(mouse_waypoint.transform.location)

            points_to_draw.append(waypoint_position)
            """
            print("is_junction: " + str(mouse_waypoint.is_junction))
            print ("lane_type :", mouse_waypoint.lane_type) # mouse_waypoint.left_lane_marking.type, mouse_waypoint.right_lane_marking.type)
            if(mouse_waypoint.is_junction):
                print("junction id: " + str(mouse_waypoint.get_junction().id))
            print("Route Yaw: " + str(mouse_waypoint.transform.rotation.yaw))
            add_route_point(mouse_waypoint.transform)

            """
            next_waypoints = mouse_waypoint.next(landmark_distance)
            print("Mouse Position: " + str(mouse_position))
            for waypoint in next_waypoints:
                position = waypoint.transform.location
                points_to_draw.append(world_to_pixel(position,
                                                pixels_per_meter, scale, world_offset,
                                                (-WIDTH / 2, -HEIGHT / 2)))
            """
            signals_position = []
            signal_waypoint_position = []
            lines_to_draw = []
            landmarks = mouse_waypoint.get_landmarks(landmark_distance, False)
            landmarks = mouse_waypoint.get_landmarks_of_type(landmark_distance, "1000001")
            print("Number Signals: " + str(len(landmarks)))
            for landmark in landmarks:
                world_position = landmark.transform.location
                screen_position = world_to_pixel(world_position,
                                                pixels_per_meter, scale, world_offset,
                                                view_offset())
                signals_position.append(screen_position)

                forward_world_position = world_position + 2 * landmark.transform.get_forward_vector()
                forward_screen_position = world_to_pixel(forward_world_position,
                                                pixels_per_meter, scale, world_offset,
                                                view_offset())
                lines_to_draw.append((screen_position, forward_screen_position))

                world_position = landmark.waypoint.transform.location
                screen_position = world_to_pixel(world_position,
                                                pixels_per_meter, scale, world_offset,
                                                view_offset())
                signal_waypoint_position.append(screen_position)
                orientation = ""
                if landmark.orientation == carla.LandmarkOrientation.Positive:
                    orientation = "Positive"
                elif landmark.orientation == carla.LandmarkOrientation.Negative:
                    orientation = "Negative"
                elif landmark.orientation == carla.LandmarkOrientation.Both:
                    orientation = "Both"
                landmark.road_id
                landmark_type = landmark.type
                if landmark.type == carla.LandmarkType.StopSign:
                    landmark_type = "STOP"
                elif landmark.type == carla.LandmarkType.YieldSign:
                    landmark_type = "YIELD"
                print(
                    "name: " + landmark.name
                    + " id: " + landmark.id
                    + " s: " + str(landmark.s)
                    + " t: " + str(landmark.t)
                    + " dynamic: " + str(landmark.is_dynamic)
                    + " orientation: " + str(orientation)
                    + " zOffset: " + str(landmark.z_offset)
                    + " country " + landmark.country
                    + " type: " + landmark_type
                    + " sub_type: " + landmark.sub_type
                    + " value: " + str(landmark.value)
                    + " height: " + str(landmark.height)
                    + " width: " + str(landmark.width)
                    + " text: " + landmark.text
                    + " hOffset: " + str(landmark.h_offset)
                    + " pitch: " + str(landmark.pitch)
                    + " roll: " + str(landmark.roll)
                    + " distance: " + str(landmark.distance)
                    + " validities: "
                    )
                validities = landmark.get_lane_validities()
                for validity in validities:
                    print("from: " + str(validity[0]) + " to " + str(validity[1]))
                landmarks_in_group = carla_map.get_landmark_group(landmark)
                out_str = "Landmarks in group: "
                for land in landmarks_in_group:
                    out_str = out_str + land.id + " "
                print(out_str)
        # ── Draw route polyline + waypoint dots ──────────────────────────────
        screen_pts = [
            world_to_pixel(loc, pixels_per_meter, scale, world_offset, view_offset())
            for loc in points_to_draw
        ]
        if len(screen_pts) >= 2:
            pygame.draw.lines(display, COLOR_GREEN, False, screen_pts, 2)
        for i, px in enumerate(screen_pts):
            pygame.draw.circle(display, COLOR_GREEN, px, 5)
            # index label next to each dot
            lbl = font.render(str(i), True, COLOR_WHITE)
            display.blit(lbl, (px[0] + 7, px[1] - 7))

        for signal_point in signals_position:
            pygame.draw.circle(display, COLOR_WHITE, signal_point, 1)
        for signal_point in signal_waypoint_position:
            pygame.draw.circle(display, COLOR_RED, signal_point, 5)
        for line in lines_to_draw:
            pygame.draw.line(display, COLOR_GREEN, line[0], line[1])

        waypoint_position = world_to_pixel(mouse_waypoint.transform.location,
                                           pixels_per_meter, scale, world_offset,
                                           view_offset())

        pygame.draw.line(display, COLOR_WHITE, mouse, waypoint_position, 1)
        pygame.draw.circle(display, COLOR_GREEN, waypoint_position, 5)

        # ── Route distance ────────────────────────────────────────────────────
        total_dist_m = sum(
            math.sqrt(
                (points_to_draw[i].x - points_to_draw[i-1].x) ** 2 +
                (points_to_draw[i].y - points_to_draw[i-1].y) ** 2
            )
            for i in range(1, len(points_to_draw))
        )
        dist_str = f"{total_dist_m:.0f} m" if total_dist_m < 1000 else f"{total_dist_m/1000:.2f} km"
        time_30 = total_dist_m / (30 / 3.6)
        time_50 = total_dist_m / (50 / 3.6)

        # ── HUD: zoom + route stats + controls ────────────────────────────────
        zoom_pct = int(scale / base_scale * 100)
        hud_lines = [
            f"Zoom: {zoom_pct}%",
            f"Waypoints: {len(points_to_draw)}   Distance: {dist_str}",
            f"Est. time: {time_30/60:.1f} min @ 30 km/h  |  {time_50/60:.1f} min @ 50 km/h",
            "Scroll/+-: zoom   MMB: pan   0/R: reset   LMB: add   RMB: save+stop",
        ]
        for i, text in enumerate(hud_lines):
            surf = font.render(text, True, COLOR_WHITE)
            bg = pygame.Surface((surf.get_width() + 6, surf.get_height() + 2))
            bg.set_alpha(140)
            bg.fill(COLOR_BLACK)
            display.blit(bg,   (4, 4 + i * 18))
            display.blit(surf, (7, 5 + i * 18))

        pygame.display.flip()

    pygame.quit()

    for point in marked_route:
        yaw_attr = f' yaw="{point[1].yaw:.2f}"' if include_yaw else ''
        print(f'<position x="{point[0].x:.2f}" y="{point[0].y:.2f}" z="{point[0].z:.2f}"{yaw_attr}/>')


if __name__ == '__main__':
    main()