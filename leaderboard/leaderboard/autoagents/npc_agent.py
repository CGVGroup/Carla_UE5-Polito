#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an NPC agent to control the ego vehicle
"""

from __future__ import print_function

import carla
from agents.navigation.basic_agent2 import BasicAgent2
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

def get_entry_point():
    return 'NpcAgent'

class NpcAgent(AutonomousAgent):

    """
    NPC autonomous agent to control the ego vehicle
    """

    _agent = None
    _route_assigned = False

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.track = Track.SENSORS

        self._agent = None
        self._failing_behavior = False
        self._failtriggers = []
        self._llmtriggers = []

    def set_failtriggers(self, failtriggers):
        """
        Set fail triggers for the agent
        """
        self._failtriggers = failtriggers

    def set_llmtriggers(self, llmtrigger):
        """
        Set llm triggers for the agent
        """
        self._llmtriggers = llmtrigger

    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]
        """

        sensors = [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation. 
        """
        if not self._agent:

            # Search for the ego actor
            hero_actor = None
            for actor in CarlaDataProvider.get_world().get_actors():
                if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                    hero_actor = actor
                    break

            if not hero_actor:
                return carla.VehicleControl()

            # Add an agent that follows the route to the ego
            self._agent = BasicAgent2(hero_actor, 20)

            plan = []
            prev_wp = None
            # for transform, _ in self._global_plan_world_coord:
            #     wp = CarlaDataProvider.get_map().get_waypoint(transform.location)
            #     # print("Adding waypoint:", wp)
            #     if prev_wp:
            #         plan.extend(self._agent.trace_route(prev_wp, wp))
            #     prev_wp = wp
            # print("Plan :", plan)
            route_length_m = 0.0
            for transform, indication in self.route:
                if prev_wp is not None:
                    route_length_m += prev_wp.transform.location.distance(transform.location)
                wp = CarlaDataProvider.get_map().get_waypoint(transform.location)
                plan.append((wp, indication))
                prev_wp = wp
            print("route : ", self.route)
            print("Plan length (m):", route_length_m)
            self._agent.set_global_plan(plan)

            return carla.VehicleControl()

        else:
            # scenario_name = CarlaDataProvider.get_current_scenario_name()
            # print("Current scenario:", scenario_name)

            print("Fail triggers:", self._failtriggers)
            print("LLM triggers:", self._llmtriggers)
            ego_vehicle_position = CarlaDataProvider.get_hero_actor().get_transform().location
            if self._failtriggers != []:
                pos_0 = self._failtriggers[0]
                distance = ego_vehicle_position.distance(pos_0)
                if distance < 3.0:
                    # switch self._failing_behavior to True if it was False and vice versa
                    self._failing_behavior = not self._failing_behavior
                    # remove the first failtrigger
                    self._failtriggers.pop(0)
            
            if self._llmtriggers != []:
                pos_0, text = self._llmtriggers[0]
                distance = ego_vehicle_position.distance(pos_0)
                if distance < 3.0:
                    print("LLM Trigger reached:", text)
                    # remove the first llmtrigger
                    self._llmtriggers.pop(0)
            
            print("Failing behavior:", self._failing_behavior)
            return self._agent.run_step(self._failing_behavior)
