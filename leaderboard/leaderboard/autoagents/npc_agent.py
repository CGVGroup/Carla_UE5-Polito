#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an NPC agent to control the ego vehicle
"""

from __future__ import print_function

import csv
import glob
import os
import time

import carla
from agents.navigation.basic_agent2 import BasicAgent2
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from leaderboard.utils.session_logger import SessionLogger
from leaderboard.utils import step_timing

def get_entry_point():
    return 'NpcAgent'


# Seconds after the gaze recording starts when it is checked for eye data (see check_gaze_recording)
GAZE_CHECK_DELAY = 8.0

# Where the CARLA server writes the gaze recordings: packaged CARLA, CARLA run from the editor
GAZE_LOG_DIRS = [
    os.path.join(os.environ.get('LOCALAPPDATA', ''), 'CarlaUnreal', 'Saved', 'GazeLogs'),
    os.path.join(os.environ.get('CARLA_UE_ROOT', r'C:\Users\USER\Documents\GitHub\Carla_UE5-Polito'),
                 'Unreal', 'CarlaUnreal', 'Saved', 'GazeLogs'),
]


def check_gaze_recording(session_name, logger=None):
    """
    Early in the route, warns loudly when the gaze recording gets no eye data (the headset sends none to
    CARLA), instead of finding out at the end of the route. Only when the server runs on this PC
    """
    files = [f for d in GAZE_LOG_DIRS for f in glob.glob(os.path.join(d, session_name + '_*.csv'))]
    if not files:
        return  # server on another PC, or not recording
    newest = max(files, key=os.path.getmtime)
    if time.time() - os.path.getmtime(newest) > 60.0:
        return  # not this route's
    try:
        with open(newest, newline='', encoding='utf-8', errors='replace') as f:
            rows = list(csv.DictReader(f))
    except OSError as e:
        print('Gaze recording not checked: CARLA keeps the file locked while recording (rebuild CARLA to allow '
              'it) ({})'.format(e))
        return
    if not rows:
        return
    valid = sum(r.get('gaze_valid') == '1' for r in rows)
    connected = any(r.get('eye_tracker_connected') == '1' for r in rows)
    if logger is not None:
        logger.event('gaze_check', samples=len(rows), valid=valid, eye_tracker_connected=connected)
    if valid:
        print('Gaze recording OK: {:.0%} valid samples so far'.format(valid / len(rows)))
        return
    print('\n' + '!' * 100)
    print('!!! NO EYE DATA in the gaze recording ({} samples, eye tracker {}): stop the route if you need '
          'the gaze.'.format(len(rows), 'seen by CARLA' if connected else 'NOT seen by CARLA'))
    print('!!! CARLA keeps the VR connection it had when it started: with the headset streaming through VIVE '
          'Business Streaming\n!!! and tools\\check_eye_tracking.py showing gaze, close CARLA and start it '
          'again (.\\launch_carla.ps1).')
    print('!' * 100 + '\n')

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
        self._visualization = 'full'
        self._logger = None  # session logger (see leaderboard/utils/session_logger.py)

    def set_failtriggers(self, failtriggers):
        """
        Set fail triggers for the agent
        """
        self._failtriggers = failtriggers

    def destroy(self):
        """
        End of the route: removes the obstacles the agent showed in the VR dashboard
        """
        if self._logger is not None:
            self._logger.close()
            self._logger = None
        if self._agent is not None:
            self._agent._vr_dashboard.stop_gaze_recording()
            self._agent._vr_dashboard.clear()

    def set_visualization(self, visualization):
        """
        Set the VR dashboard visualization mode: 'full', 'none' or 'text' (route XML attribute)
        """
        self._visualization = visualization
        print("Visualization mode:", visualization)

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

        # No camera: the agent drives from the world state, and a camera renders the scene again (and
        # reads the image back) at every tick, which made the VR view judder. The speedometer is computed
        # on the client, it costs the server nothing
        sensors = [
            {'type': 'sensor.speedometer', 'id': 'SPEED'},
        ]

        return sensors

    @staticmethod
    def _trigger_reached(ego_transform, position, radius=3.0, pass_radius=8.0):
        """
        Whether the ego reached a trigger position: within `radius` of it, or driving past it (the
        position just went behind the ego, less than `pass_radius` away). The second case catches the
        triggers passed off-center, e.g. while a driver had taken over to go around an obstacle.
        """
        location = ego_transform.location
        distance = location.distance(position)
        if distance < radius:
            return True
        if distance > pass_radius:
            return False
        forward = ego_transform.get_forward_vector()
        return (position.x - location.x) * forward.x + (position.y - location.y) * forward.y < 0.0

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
            self._agent = BasicAgent2(hero_actor, 30, opt_dict={'visualization': self._visualization})

            # Session name: the routes file's, with the visualization mode (unless it already ends with it)
            routes_name = os.path.splitext(os.path.basename(os.environ.get('ROUTES', 'route')))[0]
            session_name = routes_name if routes_name.endswith('_' + self._visualization)                 else '{}_{}'.format(routes_name, self._visualization)

            # Driver's gaze recorded during the whole route (VR headset eye tracking, on the server):
            # <CARLA Saved dir>/GazeLogs/<session name>_<date>.csv (packaged CARLA: Saved dir in
            # %LOCALAPPDATA%\CarlaUnreal\Saved)
            # (not with GAZE_RECORD=0: quick tests, see run_leaderboard_xai.ps1)
            if os.environ.get('GAZE_RECORD', '1') != '0':
                self._agent._vr_dashboard.start_gaze_recording(session_name)
                self._gaze_check = (time.monotonic() + GAZE_CHECK_DELAY, session_name)

            # Everything happening during the route, for the analysis afterwards: logs/sessions/<routes
            # file>_<visualization>_<date>/ (frames.csv, events.jsonl, summary.json)
            try:
                self._logger = SessionLogger(hero_actor, session_name, self._visualization,
                                             routes_file=os.environ.get('ROUTES'))
            except (OSError, RuntimeError) as e:
                print("Session logger not started:", e)

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

            if getattr(self, '_gaze_check', None) and time.monotonic() > self._gaze_check[0]:
                check_gaze_recording(self._gaze_check[1], self._logger)
                self._gaze_check = None

            print("Fail triggers:", self._failtriggers)
            print("LLM triggers:", self._llmtriggers)
            ego_transform = CarlaDataProvider.get_hero_actor().get_transform()
            ego_vehicle_position = ego_transform.location
            if self._failtriggers != []:
                pos_0 = self._failtriggers[0]
                if self._trigger_reached(ego_transform, pos_0):
                    # switch self._failing_behavior to True if it was False and vice versa
                    self._failing_behavior = not self._failing_behavior
                    if self._logger is not None:
                        self._logger.event('failtrigger', failure_behavior=self._failing_behavior,
                                           trigger=[round(pos_0.x, 2), round(pos_0.y, 2)])
                    # remove the first failtrigger
                    self._failtriggers.pop(0)
            
            if self._llmtriggers != []:
                pos_0, text = self._llmtriggers[0]
                if self._trigger_reached(ego_transform, pos_0):
                    print("LLM Trigger reached:", text)
                    if self._logger is not None:
                        self._logger.event('llm_trigger', text=text)
                    # remove the first llmtrigger
                    self._llmtriggers.pop(0)
            
            print("Failing behavior:", self._failing_behavior)
            with step_timing.measure('planner'):
                control = self._agent.run_step(self._failing_behavior)
            if self._logger is not None:
                try:
                    with step_timing.measure('logger'):
                        self._logger.tick(self._agent.step_info, self._failing_behavior, self._agent._vr_dashboard)
                except Exception as e:  # noqa: BLE001 - logging must never stop the route
                    print("Session logger error:", e)
            return control
