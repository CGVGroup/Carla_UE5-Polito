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
from pygame.locals import *

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
import threading
import tkinter as tk
from tkinter import ttk
from tkinter import Scale
import random

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

class WeatherControlPanel:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title('Weather Control')

        # Main Frame
        self.main_frame = tk.Frame(self.window)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Cloudiness 

        self.cloudiness = tk.DoubleVar(self.window, value=50)

        tk.Label(self.main_frame, text="Cloudiness").pack(padx=10, pady=10)
        self.cloudiness_slider = Scale(self.main_frame, from_=0, to=100, orient="horizontal", variable=self.cloudiness)
        self.cloudiness_slider.pack(padx=10, pady=10)

        # Button definition
        self.generate_btn = tk.Button(self.main_frame, text="Generate", command=self.on_generate_clicked)
        self.generate_btn.pack(pady=10)

    def clear_frame(self, frame):
        # Destroy all children widgets in a given frame
        for widget in frame.winfo_children():
            widget.destroy()

    def get_cloudiness(self):
        return self.cloudiness.get()


    def on_generate_clicked(self):
        # This method will be executed when the "Generate" button is pressed
        self.clear_frame(self.main_frame)

        # Create the combobox with random propositions
        propositions = [f"{i}" for i in ["PedestrianCrossing","CarFlow", "PedestrianFlow", "TurnLeft"]]  # 20 random choices as an example
        random.shuffle(propositions)

        self.combo = ttk.Combobox(self.main_frame, values=propositions)
        self.combo.pack(pady=10)
        self.combo.set("Select a proposition")

        # Open a new window button
        open_window_btn = tk.Button(self.main_frame, text="Settings", command=self.open_new_window)
        open_window_btn.pack(pady=10)

    def open_new_window(self):
        # Get the selected choice
        selected_choice = self.combo.get()

        # Open a new window with the selected choice
        new_window = tk.Toplevel(self.window)
        new_window.title(f"{selected_choice}")
        tk.Label(new_window, text=f"{selected_choice}").pack(pady=20)

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


def add_route_point(location, ppm, scale, world_offset):
    waypoint_position = world_to_pixel(location,
                                           ppm, scale, world_offset,
                                           (-WIDTH / 2, -HEIGHT / 2))
            
    marked_route.append(location)
    points_to_draw.append(waypoint_position)


def flush_route_to_file():
    global capture_route_file
    global marked_route
    global points_to_draw

    print("flushing")
    for point in marked_route:
        capture_route_file.write(f'\t<position x="{point.x:.2f}" y="{point.y:.2f}" z="{point.z:.2f}"/>\n')
    
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

HEIGHT = 2160
WIDTH = 2160

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
    args = argparser.parse_args()

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
    scale = 0.99
    pixels_per_meter = min(WIDTH / max(1.0, road_width), HEIGHT / max(1.0, road_height))
    world_offset = road_mid

    display.fill(COLOR_BLACK)
    for waypoint in waypoints:
        point = waypoint.transform
        color = COLOR_GREEN
        if (waypoint.lane_id < 0):
            color = COLOR_YELLOW
        if (waypoint.is_junction):
            if waypoint.lane_id < 0:
                color = COLOR_BLUE
            else:
                color = COLOR_PINK
        point_screen = world_to_pixel(point.location, pixels_per_meter, scale, world_offset,
            (-WIDTH / 2, -HEIGHT / 2))
        pygame.draw.circle(display, color, point_screen, 1)
    if args.lights:
        landmarks = carla_map.get_all_landmarks_of_type("1000001")
        for landmark in landmarks:
            world_position = landmark.transform.location
            screen_position = world_to_pixel(world_position,
                                            pixels_per_meter, scale, world_offset,
                                            (-WIDTH / 2, -HEIGHT / 2))
            forward_world_position = world_position + 2 * landmark.transform.get_forward_vector()
            forward_screen_position = world_to_pixel(forward_world_position,
                                            pixels_per_meter, scale, world_offset,
                                            (-WIDTH / 2, -HEIGHT / 2))
            pygame.draw.circle(display, COLOR_WHITE, screen_position, 1)
            pygame.draw.line(display, COLOR_GREEN, screen_position, forward_screen_position)
            print("landmark " + str(landmark.id) + " at " + str(world_position))
    pygame.display.flip()
    road = display.convert()


    pygame.image.save(road, "map_viewer_image.png")
    print(str(len(waypoints)) + ' waypoints')
    waypoints = None

    landmark_distance = 10

    signals_position = []
    signal_waypoint_position = []
    lines_to_draw = []
    while True:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            break

        display.blit(road, (0, 0))

        mouse = pygame.mouse.get_pos()
        mouse_position = pixel_to_world(mouse[0], mouse[1], pixels_per_meter,
                                        scale, world_offset, (-WIDTH / 2, -HEIGHT / 2))
        mouse_waypoint = carla_map.get_waypoint(mouse_position)
        if mouse_waypoint == None:
            pygame.display.flip()
            continue


        if event.type == pygame.MOUSEBUTTONDOWN:

            if event.button == 3:
                stop_route_capturing()
                continue

            start_route_capturing()

            print("road id: " + str(mouse_waypoint.road_id) + " lane id: " + str(mouse_waypoint.lane_id))
            print("Route Yaw: " + str(mouse_waypoint.transform.rotation.yaw))

            add_route_point(mouse_waypoint.transform.location, pixels_per_meter, scale, world_offset)

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
                                                (-WIDTH / 2, -HEIGHT / 2))
                signals_position.append(screen_position)

                forward_world_position = world_position + 2 * landmark.transform.get_forward_vector()
                forward_screen_position = world_to_pixel(forward_world_position,
                                                pixels_per_meter, scale, world_offset,
                                                (-WIDTH / 2, -HEIGHT / 2))
                lines_to_draw.append((screen_position, forward_screen_position))

                world_position = landmark.waypoint.transform.location
                screen_position = world_to_pixel(world_position,
                                                pixels_per_meter, scale, world_offset,
                                                (-WIDTH / 2, -HEIGHT / 2))
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
        for point_to_draw in points_to_draw:
            pygame.draw.circle(display, COLOR_GREEN, point_to_draw, 5)
        for signal_point in signals_position:
            pygame.draw.circle(display, COLOR_WHITE, signal_point, 1)
        for signal_point in signal_waypoint_position:
            pygame.draw.circle(display, COLOR_RED, signal_point, 5)
        for line in lines_to_draw:
            pygame.draw.line(display, COLOR_GREEN, line[0], line[1])

        waypoint_position = world_to_pixel(mouse_waypoint.transform.location,
                                           pixels_per_meter, scale, world_offset,
                                           (-WIDTH / 2, -HEIGHT / 2))
        
        
        pygame.draw.line(display, COLOR_WHITE, mouse, waypoint_position, 1)
        
        
        pygame.draw.circle(display, COLOR_GREEN, waypoint_position, 5)
        
        
        pygame.display.flip()

    pygame.quit()

    for point in marked_route:
        print(f'<position x="{point.x}" y="{point.y}" z="{point.z}"/>')


if __name__ == '__main__':
    # main()
    carla_thread = threading.Thread(target=main)
    carla_thread.start()

    # Create and run the WeatherControlPanel in the main thread
    panel = WeatherControlPanel()
    panel.window.mainloop()