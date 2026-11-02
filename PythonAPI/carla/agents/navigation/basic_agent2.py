# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specifed route
"""

import math
import types

import carla
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.geometry import MultiPoint
from agents.navigation.local_planner import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.decision_display import get_display
from agents.navigation.vr_dashboard import get_dashboard, is_parked_vehicle, LevelObject, PARKED_CAR_FIRST_ID
from agents.tools.misc import (get_speed, is_within_distance,
                               get_trafficlight_trigger_location,
                               compute_distance)


# Measured steering response of the ego vehicle models: (wheelbase [m], [(steer, curvature [1/m]), ...])
# at ~11 km/h, measured with PythonAPI/examples/measure_steering.py (it prints the line to put here)
STEERING_CALIBRATIONS = {
    'vehicle.tesla.vr': (2.9, [(0.0, 0.0), (0.2, 0.0079), (0.4, 0.0308), (0.6, 0.0676),
                               (0.8, 0.1274), (1.0, 0.2050)]),
}
_REFERENCE_CALIBRATION = 'vehicle.tesla.vr'


def steering_calibration(vehicle):
    """
    (wheelbase, steer -> curvature map) of a vehicle for the pure pursuit steering: its measured one in
    STEERING_CALIBRATIONS if any. Otherwise an estimate: the reference model's (measured) curve, assuming
    the same wheel angles for the same steer (the VR vehicles share their steering setup), scaled by the
    wheelbases (for a given wheel angle, the curvature is inversely proportional to the wheelbase; this
    vehicle's from its physics)
    """
    model = vehicle.type_id.lower()
    if model in STEERING_CALIBRATIONS:
        return STEERING_CALIBRATIONS[model]
    wheelbase, curve = STEERING_CALIBRATIONS[_REFERENCE_CALIBRATION]
    try:
        wheels = vehicle.get_physics_control().wheels
        front = [w for w in wheels if w.max_steer_angle > 0.1] or wheels[:2]
        rear = [w for w in wheels if w not in front] or wheels[2:]
        def center(ws):
            return carla.Location(x=sum(w.position.x for w in ws) / len(ws),
                                  y=sum(w.position.y for w in ws) / len(ws),
                                  z=sum(w.position.z for w in ws) / len(ws))
        distance = center(front).distance(center(rear))
        own_wheelbase = distance / 100.0 if distance > 20.0 else distance  # wheel positions in cm or m
        max_angle = max(w.max_steer_angle for w in front)
        if not 1.5 < own_wheelbase < 5.0 or not 10.0 < max_angle < 70.0:
            raise ValueError('wheelbase {:.2f} m, max steer angle {:.1f} deg'.format(own_wheelbase, max_angle))
        # Same wheel angles, other wheelbase (bicycle model: curvature = tan(wheel angle) / wheelbase)
        scale = min(max(wheelbase / own_wheelbase, 0.5), 2.0)
        estimate = [(s, k * scale) for s, k in curve]
        print("Steering of {}: no measured calibration, estimated (wheelbase {:.2f} m, max wheel angle {:.1f} deg: "
              "x{:.2f} the {} curve). Measure it with PythonAPI/examples/measure_steering.py".format(
                  vehicle.type_id, own_wheelbase, max_angle, scale, _REFERENCE_CALIBRATION))
        return own_wheelbase, estimate
    except (RuntimeError, ValueError, ZeroDivisionError, IndexError, AttributeError) as e:
        print("Steering of {}: no measured calibration, using the {} one ({})".format(
            vehicle.type_id, _REFERENCE_CALIBRATION, e))
        return wheelbase, curve


class BasicAgent2(object):
    """
    BasicAgent implements an agent that navigates the scene.
    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    def __init__(self, vehicle, target_speed=20, opt_dict={}, map_inst=None, grp_inst=None):
        """
        Initialization the agent paramters, the local and the global planner.

            :param vehicle: actor to apply to agent logic onto
            :param target_speed: speed (in Km/h) at which the vehicle will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the LocalPlanner.
            :param map_inst: carla.Map instance to avoid the expensive call of getting it.
            :param grp_inst: GlobalRoutePlanner instance to avoid the expensive call of getting it.

        """
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        if map_inst:
            if isinstance(map_inst, carla.Map):
                self._map = map_inst
            else:
                print("Warning: Ignoring the given map as it is not a 'carla.Map'")
                self._map = self._world.get_map()
        else:
            self._map = self._world.get_map()
        self._last_traffic_light = None

        # Base parameters
        self._ignore_traffic_lights = False
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._ignore_pedestrians = False
        self._use_bbs_detection = False
        self._target_speed = target_speed
        self._sampling_resolution = 2.0
        self._base_tlight_threshold = 2.0  # meters
        self._base_vehicle_threshold = 10.0  # meters
        self._speed_ratio = 1
        self._max_brake = 0.5
        self._offset = 0
        self._left_turn_committed = False
        # Dashboard (AR-HUD): what the ego takes into account this tick, besides its main hazards
        self._oncoming_watch = {}  # vehicle id -> (vehicle, is_danger)
        self._construction_watch = []  # [(prop actor, is_danger)]
        self._oncoming_watch_horizon = 4.0  # [s] oncoming vehicles reaching a left turn's conflict zone within it are shown
        self._left_turn_block_start = None
        self._left_turn_max_wait = 8.0  # seconds blocked before forcing a commit, avoids a permanent livelock

        # Curve speed control: the local planner otherwise holds the cruise speed through
        # every turn, so the PID controller can't follow tight curves and swings wide.
        self._max_lateral_accel = 2.0   # m/s^2 allowed in curves (comfortable driving)
        self._comfort_decel = 2.5       # m/s^2 used to start slowing down before a curve
        self._min_curve_speed = 10.0    # km/h
        self._curve_lookahead = 30.0    # m of plan checked for upcoming curves
        self._curve_sample_step = 3.0   # m between the points used to estimate curvature

        # Bypass (construction / stalled vehicle): drive along the center of the opposite
        # lane and slide into / out of it gradually instead of jumping sideways.
        self._bypass_lane_offset = 1.0  # in lane widths, to the left
        self._bypass_speed = 30.0       # km/h, max speed while driving around an obstacle (same as cruise: no slowing down)
        # Side shifts (lateral offset changes) follow an S-curve over the distance driven: at most
        # _offset_max_slope sideways meters per meter driven (~7 deg), the slope changing by at most
        # _offset_slope_rate per meter. Moving one lane over (~3.5 m) takes ~15 m, at any speed.
        # While shifting, the speed is limited so that the lateral acceleration stays under
        # _shift_max_lateral_accel (~15 km/h with these values): a short shift, but a smooth one.
        # The 'tight' values are only used when there isn't room for that, at low speed.
        self._offset_max_slope = 0.4
        self._offset_slope_rate = 0.1  # [1/m]
        self._shift_max_lateral_accel = 1.8  # [m/s2]
        # The side shift around an obstacle starts this much earlier than needed to be done right when
        # reaching it: the ego stays in its lane until ~14 m before the first obstacle (the warning panel,
        # ~5 m before the cones), not as soon as the obstacle is seen
        self._shift_start_margin = 1.0  # [m]
        self._offset_tight_slope = 0.6
        self._offset_tight_slope_rate = 0.15  # [1/m] (95% of a lane-wide shift within ~9 m)
        self._tight_shift_speed = 12.0  # [km/h]
        self._offset_slope = 0.0
        self._tight_side_shift = False
        self._bypass_target = None  # offset held while driving around an obstacle
        # Stop signs: the ego makes a short full stop at each one on its route, then goes on
        self._stop_sign_lookahead = 40.0  # [m]
        self._stop_sign_wait = 2.0  # [s] time standing still at the sign
        self._stop_sign_gap = 1.0  # [m] between the ego's front and the sign when stopped
        self._stop_sign_creep_speed = 5.0  # [km/h] slowest approach speed before the final stop
        self._stop_sign_state = None  # [landmark id, time the ego stopped (or None)]
        self._stop_signs_done = set()  # ids of the signs already stopped at, until passed
        self._bypass_started = False  # whether the lane change around the obstacle has started
        self._bypass_target_tolerance = 0.25  # [m] smaller changes of the computed offset are ignored
        self._obstacle_wait_gap = 14.0   # [m] Where to wait before an obstacle that can't be bypassed yet (room for a tight side shift)
        self._obstacle_return_gap = 6.0  # [m] How far past an obstacle (behind the ego's rear) before getting back
        # Lane changes of the route are driven as smooth S-curves over this distance [m] (see _lane_change_offset)
        self._lane_change_length = 35.0
        self._plan_s = []
        self._lane_changes = []
        self._static_obstacle_margin = 0.5     # m kept between the ego's side and a static prop
        self._static_obstacle_min_margin = 0.3 # m, used only when the road is too narrow for the above
        self._static_obstacle_lookahead = 70.0 # m of plan checked for static props (the smooth side shift needs ~43 m)
        self._level_obstacles = []             # fixed level objects near the route (see set_global_plan)
        self._level_parked_cars = []           # parked cars of the level near the route ('full' mode boxes)
        self._dt = self._world.get_settings().fixed_delta_seconds or 0.05

        # Diagnostics: track whether construction props ever exist in the world
        # and how close the ego gets to the nearest one.
        self._last_prop_count = None
        self._prop_obstacle_cache = {}  # prop actor id -> (step it was computed at, obstacle description)
        self._prop_obstacle_step = 0
        self._min_prop_distance_seen = float('inf')

        # Pedestrians: last seen locations (to estimate their motion), the margin added to the
        # ego's half width to define its path, and how far ahead [s] a pedestrian walking
        # towards that path counts as entering it
        self._ped_last_seen = {}
        self._pedestrian_margin = 1.0
        self._pedestrian_horizon = 3.0

        # Diagnostics: whether the ego is currently in a lane-offset excursion (see run_step),
        # and the peak signed offset reached during it.
        self._lane_offset_exceeded = False
        self._lane_offset_peak = 0.0

        # Decision display
        self._display = get_display()
        self._hazard_sources: set = set()
        self._construction_bypass_active: bool = False
        self._last_now_msg: str = ""

        # Stalled lead-vehicle bypass: same idea as the left-turn commit timeout above,
        # but for a vehicle stopped dead ahead in our own lane (e.g. another car stuck at
        # a junction) - without this, a stalled lead vehicle blocks the ego forever.
        self._stalled_vehicle_id = None
        self._stalled_vehicle_block_start = None
        self._stalled_vehicle_max_wait = 6.0  # seconds before treating the lead vehicle as static
        self._vehicle_bypass_active: bool = False

        # VR dashboard bridge, in the route's visualization mode ('full': ego speed/steer + boxes on the
        # obstacles taken into account, 'none': nothing, 'text': only a sentence telling what the ego is
        # doing and why, see _narrate)
        self._vr_dashboard = get_dashboard(self._world, opt_dict.get('visualization', 'full'))
        self._narration_key = None  # what the last sentence was about (see _narrate)
        self.step_info = None  # what the agent perceived and decided at its last step (see run_step)
        self._narration_since = 0.0  # since when [s, simulation time]
        self._narration_resume = None  # (sentence, until when) said when moving off after a stop
        self._narration_was_stopped = False  # whether the ego stopped during the current sentence
        self._last_lane_change_dir = None  # 'left' / 'right': the route's last lane change direction

        # Change parameters according to the dictionary
        opt_dict['target_speed'] = target_speed
        if 'ignore_traffic_lights' in opt_dict:
            self._ignore_traffic_lights = opt_dict['ignore_traffic_lights']
        if 'ignore_stop_signs' in opt_dict:
            self._ignore_stop_signs = opt_dict['ignore_stop_signs']
        if 'ignore_vehicles' in opt_dict:
            self._ignore_vehicles = opt_dict['ignore_vehicles']
        if 'ignore_pedestrians' in opt_dict:
            self._ignore_pedestrians = opt_dict['ignore_pedestrians']
        if 'use_bbs_detection' in opt_dict:
            self._use_bbs_detection = opt_dict['use_bbs_detection']
        if 'sampling_resolution' in opt_dict:
            self._sampling_resolution = opt_dict['sampling_resolution']
        if 'base_tlight_threshold' in opt_dict:
            self._base_tlight_threshold = opt_dict['base_tlight_threshold']
        if 'base_vehicle_threshold' in opt_dict:
            self._base_vehicle_threshold = opt_dict['base_vehicle_threshold']
        if 'detection_speed_ratio' in opt_dict:
            self._speed_ratio = opt_dict['detection_speed_ratio']
        if 'max_brake' in opt_dict:
            self._max_brake = opt_dict['max_brake']
        if 'offset' in opt_dict:
            self._offset = opt_dict['offset']

        # Steering: pure pursuit through the measured steering response of the ego (its model's entry in
        # STEERING_CALIBRATIONS, see there). The response is far from linear (vehicle.tesla.vr: steer 0.4
        # only turns the wheels ~5 deg, 0.8 ~20 deg), which made the PID controller swing wide in every
        # turn. The full steering range is needed for tight junction turns.
        if 'lateral_control_dict' not in opt_dict:
            wheelbase, steer_curvature_map = steering_calibration(self._vehicle)
            opt_dict['lateral_control_dict'] = {
                'type': 'pure_pursuit',
                'wheelbase': wheelbase,
                'steer_curvature_map': steer_curvature_map,
            }
        if 'max_steering' not in opt_dict:
            opt_dict['max_steering'] = 1.0
        # Shorter lookahead than the planner's default (3 m + 0.3 s): pure pursuit cuts
        # corners by roughly the lookahead, which on junction S-bends put the ego up to
        # 0.65 m off the lane center and into bollards / curbs at the lane edge
        if 'base_min_distance' not in opt_dict:
            opt_dict['base_min_distance'] = 1.5
        if 'distance_ratio' not in opt_dict:
            opt_dict['distance_ratio'] = 0.2

        # Initialize the planners
        self._local_planner = LocalPlanner(self._vehicle, opt_dict=opt_dict, map_inst=self._map)
        if grp_inst:
            if isinstance(grp_inst, GlobalRoutePlanner):
                self._global_planner = grp_inst
            else:
                print("Warning: Ignoring the given map as it is not a 'carla.Map'")
                self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)
        else:
            self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)

        # Get the static elements of the scene
        self._lights_list = self._world.get_actors().filter("*traffic_light*")
        self._lights_map = {}  # Dictionary mapping a traffic light to a wp corrspoing to its trigger volume location
        # Traffic lights are matched to the route through their stop lines (see _traffic_light_control)
        self._light_stop_wps = None  # [(traffic light, [stop line waypoints])], computed on first use
        self._traffic_light_lookahead = 40.0  # [m]
        self._traffic_light_gap = 0.5  # [m] between the ego's front and the stop line when stopped
        self._traffic_light_creep_speed = 5.0  # [km/h] slowest approach speed before the final stop
        self._yellow_max_decel = 3.5  # [m/s2] stops at a yellow light only if it can with this deceleration
        self._light_refresh_period = 2.0  # [s] the list of lights is refreshed (large maps add them later)
        self._light_refresh_time = None
        self._light_diag_done = set()  # lights already reported by the diagnostics
        self._stop_line_junction_dist = 20.0  # [m] a stop line counts only if a junction follows it within this

    def add_emergency_stop(self, control):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed):
        """
        Changes the target speed of the agent
            :param speed (float): target speed in Km/h
        """
        self._target_speed = speed
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value=True):
        """
        If active, the agent will dynamically change the target speed according to the speed limits

            :param value (bool): whether or not to activate this behavior
        """
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self):
        """Get method for protected member local planner"""
        return self._local_planner

    def get_global_planner(self):
        """Get method for protected member local planner"""
        return self._global_planner

    def set_destination(self, end_location, start_location=None):
        """
        This method creates a list of waypoints between a starting and ending location,
        based on the route returned by the global router, and adds it to the local planner.
        If no starting location is passed, the vehicle local planner's target location is chosen,
        which corresponds (by default), to a location about 5 meters in front of the vehicle.

            :param end_location (carla.Location): final location of the route
            :param start_location (carla.Location): starting location of the route
        """
        if not start_location:
            start_location = self._local_planner.target_waypoint.transform.location
            clean_queue = True
        else:
            start_location = self._vehicle.get_location()
            clean_queue = False

        start_waypoint = self._map.get_waypoint(start_location)
        end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self.trace_route(start_waypoint, end_waypoint)
        self._local_planner.set_global_plan(route_trace, clean_queue=clean_queue)

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        """
        Adds a specific plan to the agent.

            :param plan: list of [carla.Waypoint, RoadOption] representing the route to be followed
            :param stop_waypoint_creation: stops the automatic random creation of waypoints
            :param clean_queue: resets the current agent's plan
        """
        self._level_obstacles = self._find_level_obstacles(plan)
        self._level_parked_cars = self._find_parked_cars(plan)
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue
        )
        self._find_lane_changes(plan if clean_queue else None)
        # 'full' visualization: the route's global path in the AR-HUD, as the ego drives it: with the plan's
        # lane changes smoothed into the same S-curves (the plan itself jumps from lane to lane in a few
        # meters, which drawn as is cuts across the road)
        if clean_queue:
            self._vr_dashboard.set_path(self._smoothed_plan(plan))

    def _find_lane_changes(self, plan):
        """
        Finds the places where the plan jumps sideways to a neighbor lane. The route planner switches
        lanes from one waypoint to the next (2 to 4 m sideways within a few meters), which would make
        the ego swerve; _lane_change_offset turns them into smooth S-curves.
        Stores the distance along the plan of each waypoint, and the lane changes as groups
        (curve start, curve end, [(distance, sideways jump), ...]).

        Jumps closer than the S-curve length are grouped and driven as a single curve, going straight
        to the lane the group ends in. E.g. the route sometimes moves to the next lane and back a few
        meters later: the ego then simply stays in its lane, instead of a quick left-right swerve.
        """
        self._plan_s = []
        self._lane_changes = []
        if not plan:
            return
        jumps = []
        dist = 0.0
        prev_wp = None
        for wp, _ in plan:
            if prev_wp is not None:
                vec = wp.transform.location - prev_wp.transform.location
                forward = prev_wp.transform.get_forward_vector()
                right = prev_wp.transform.get_right_vector()
                along = vec.x * forward.x + vec.y * forward.y
                sideways = vec.x * right.x + vec.y * right.y
                if abs(sideways) > 1.0 and abs(sideways) > 0.3 * abs(along):
                    jumps.append((dist + max(along, 0.0) / 2.0, sideways))
                dist += max(along, 0.0)
            self._plan_s.append(dist)
            prev_wp = wp

        half = self._lane_change_length / 2.0
        for s_jump, sideways in jumps:
            if self._lane_changes and s_jump - half < self._lane_changes[-1][1]:
                start, _, group = self._lane_changes[-1]
                group.append((s_jump, sideways))
                self._lane_changes[-1] = (start, s_jump + half, group)
            else:
                self._lane_changes.append((s_jump - half, s_jump + half, [(s_jump, sideways)]))

    def trace_route(self, start_waypoint, end_waypoint):
        """
        Calculates the shortest route between a starting and ending waypoint.

            :param start_waypoint (carla.Waypoint): initial waypoint
            :param end_waypoint (carla.Waypoint): final waypoint
        """
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        return self._global_planner.trace_route(start_location, end_location)

    def run_step(self, failure_behavior=False):
        """Execute one step of navigation."""
        hazard_detected = False
        new_hazard_sources = set()
        construction_offset = None  # set when bypassing construction
        desired_offset = None  # lateral offset to move towards this tick (None keeps the current one)
        self._oncoming_watch = {}  # filled by the oncoming traffic checks of this tick

        actors = self._world.get_actors()  # once per step (filtered locally)
        vehicle_list = actors.filter("*vehicle*")
        pedestrian_list = actors.filter("*walker*")
        prop_list = actors.filter("static.prop.trafficwarning")

        vehicle_speed = get_speed(self._vehicle) / 3.6
        speed_kmh = get_speed(self._vehicle)
        ego_location = self._vehicle.get_location()
        ego_wpt = self._map.get_waypoint(ego_location)

        # Diagnostics: flag when the ego drifts more than a quarter lane width from the
        # lane center, to quantify/localize lane-following drift rather than guessing.
        # Also tags the maneuver (road option + local planner's lateral offset) and
        # tracks the peak deviation reached during the excursion, not just the
        # threshold-crossing moments, so "how bad" is actually measurable.
        lane_vec = ego_wpt.transform.location - ego_location
        right_vec = ego_wpt.transform.get_right_vector()
        lateral_offset = lane_vec.x * right_vec.x + lane_vec.y * right_vec.y
        exceed_threshold = 0.25 * ego_wpt.lane_width
        recover_threshold = 0.15 * ego_wpt.lane_width
        road_option = self._local_planner.target_road_option
        if abs(lateral_offset) > exceed_threshold and not self._lane_offset_exceeded:
            print("DIAGNOSTIC: lane offset {:.2f} m (> {:.2f} m threshold, lane_width={:.2f} m) at {} "
                  "road_id={} lane_id={} road_option={} planner_offset={:.2f}".format(
                      lateral_offset, exceed_threshold, ego_wpt.lane_width, ego_location,
                      ego_wpt.road_id, ego_wpt.lane_id, road_option, self._offset))
            self._lane_offset_exceeded = True
            self._lane_offset_peak = lateral_offset
        elif self._lane_offset_exceeded and abs(lateral_offset) > abs(self._lane_offset_peak):
            self._lane_offset_peak = lateral_offset
        if abs(lateral_offset) <= recover_threshold and self._lane_offset_exceeded:
            print("DIAGNOSTIC: lane offset recovered ({:.2f} m, peak was {:.2f} m) at {}".format(
                lateral_offset, self._lane_offset_peak, ego_location))
            self._lane_offset_exceeded = False

        # Detect vehicle obstacles
        max_vehicle_distance = self._base_vehicle_threshold + self._speed_ratio * vehicle_speed
        affected_by_vehicle, obstacle_vehicle, _ = self._vehicle_obstacle_detected(vehicle_list, max_vehicle_distance)

        # Detect traffic light: red (or yellow) lights whose stop line is on the route ahead
        light_speed_limit, affected_by_tlight, obstacle_tlight = self._traffic_light_control(
            ego_location, vehicle_speed)
        if affected_by_tlight:
            hazard_detected = True
        if obstacle_tlight is not None:
            new_hazard_sources.add('traffic_light')

        # Stop signs: short full stop
        stop_sign_speed_limit, stop_at_sign = self._stop_sign_control(ego_wpt, vehicle_speed)
        if stop_at_sign:
            hazard_detected = True
            new_hazard_sources.add('stop_sign')

        # Detect pedestrians
        max_pedestrian_distance = 15.0
        affected_by_pedestrian, obstacle_pedestrian = self._pedestrian_obstacle_detected(pedestrian_list, max_pedestrian_distance)
        if affected_by_pedestrian and not failure_behavior:
            hazard_detected = True
            new_hazard_sources.add('pedestrian')

        if len(prop_list) != self._last_prop_count:
            print("DIAGNOSTIC: static.prop.trafficwarning count in world changed: {} -> {} (locations: {})".format(
                self._last_prop_count, len(prop_list), [p.get_location() for p in prop_list]))
            self._last_prop_count = len(prop_list)

        # Static obstacle avoidance (construction cones and signs, barriers, any static prop):
        # shift just enough to clear the props that are actually on the path
        # Failing behavior (route failtriggers): construction zones are ignored, the ego drives into them
        if failure_behavior:
            static_obstacles = []
        else:
            static_obstacles = self._prop_obstacles(actors.filter("static.prop.*"))
            static_obstacles += self._level_obstacles
        avoid_offset, obstacle_prop, obstacle_room, nudge_offset, needs_lane_change = \
            self._static_obstacle_avoidance(ego_location, vehicle_speed, static_obstacles)
        construction_detected = obstacle_room is not None
        obstacle_speed_limit = None  # [km/h]
        construction_phase = None  # for the narration: 'wait', 'blocked', 'approach', 'bypass', 'nudge', 'vehicle_bypass'

        if construction_detected:
            ego_half_width = self._vehicle.bounding_box.extent.y
            invades_opposite = (avoid_offset is not None
                                and avoid_offset - ego_half_width < -ego_wpt.lane_width / 2)
            blocked = avoid_offset is None
            if not blocked and invades_opposite:
                bypass_length = min(self._static_obstacle_lookahead, obstacle_room + 35.0)
                blocked = self._oncoming_blocks_bypass(ego_location, vehicle_list, bypass_length)
            if blocked:
                construction_phase = 'wait' if avoid_offset is not None else 'blocked'
                # Wait for it to clear close to the obstacle, like a driver would: keep driving in the
                # lane (back to its center if the side shift had started) and stop a few meters before it
                self._construction_bypass_active = False
                self._bypass_target = None
                self._bypass_started = False
                self._tight_side_shift = False
                desired_offset = nudge_offset
                stop_room = obstacle_room - self._obstacle_wait_gap
                if stop_room <= 0.5:
                    hazard_detected = True
                else:
                    obstacle_speed_limit = math.sqrt(2.0 * self._comfort_decel * stop_room) * 3.6
                if stop_room < 20.0:
                    new_hazard_sources.add('construction')
            else:
                # Small changes of the computed offset (obstacles entering / leaving the lookahead)
                # are ignored, so the ego holds a steady line next to them instead of wobbling
                if self._bypass_target is None or abs(avoid_offset - self._bypass_target) > self._bypass_target_tolerance:
                    self._bypass_target = avoid_offset
                construction_offset = self._bypass_target
                remaining_shift = abs(construction_offset - self._offset)
                start_room = self._shift_distance(remaining_shift, tight=False) + self._shift_start_margin
                if needs_lane_change and not self._bypass_started:
                    if obstacle_room > start_room:
                        # Still far from the obstacle that needs the lane change (obstacle_room is
                        # measured to it, not to small ones only needing a nudge): keep the lane, with
                        # just those nudges. The side shift starts just early enough to be done right
                        # before the obstacle. Slow down to the side shift speed just before it starts
                        construction_offset = nudge_offset
                        if obstacle_room < start_room + 15.0:
                            obstacle_speed_limit = self._shift_speed()
                    else:
                        self._bypass_started = True
                        if obstacle_room < self._shift_distance(remaining_shift, tight=False):
                            # Too close for the gentle side shift (e.g. after waiting near the obstacle):
                            # a steeper one, at low speed. Kept until the shift is done (switching between
                            # the two profiles in the middle of it would make the ego overshoot and wobble)
                            self._tight_side_shift = True
                if needs_lane_change and self._bypass_started and not self._construction_bypass_active:
                    self._display.log(
                        f"Obstacle bypass — lateral offset {construction_offset:.2f} m", 'action')
                    self._construction_bypass_active = True
                desired_offset = construction_offset
                if self._tight_side_shift:
                    obstacle_speed_limit = self._tight_shift_speed
                if needs_lane_change:
                    construction_phase = 'bypass' if self._bypass_started else 'approach'
                elif abs(construction_offset) > 0.1:
                    construction_phase = 'nudge'
        else:
            self._bypass_target = None
            self._bypass_started = False
            self._tight_side_shift = False
            if self._construction_bypass_active:
                self._display.log("Construction zone cleared — resuming normal lane", 'info')
                self._construction_bypass_active = False

            # Stalled lead-vehicle bypass: a vehicle directly ahead in our own lane that
            # hasn't moved in _stalled_vehicle_max_wait seconds is treated as a static
            # obstacle (e.g. another car wedged/stuck at a junction) rather than something
            # to wait behind indefinitely - mirrors the construction bypass above.
            vehicle_bypass_offset = None
            if affected_by_vehicle and obstacle_vehicle is not None and not ego_wpt.is_junction:
                now = self._world.get_snapshot().timestamp.elapsed_seconds
                if obstacle_vehicle.id != self._stalled_vehicle_id:
                    self._stalled_vehicle_id = obstacle_vehicle.id
                    self._stalled_vehicle_block_start = now
                elif (get_speed(obstacle_vehicle) < 0.5
                        and now - self._stalled_vehicle_block_start > self._stalled_vehicle_max_wait):
                    oncoming_blocked = self._oncoming_blocks_bypass(ego_location, vehicle_list, 25.0)
                    if not oncoming_blocked:
                        vehicle_bypass_offset = -self._bypass_lane_offset * ego_wpt.lane_width
                        if not self._vehicle_bypass_active:
                            self._display.log(
                                f"Lead vehicle stalled {self._stalled_vehicle_max_wait:.0f}s+ — bypassing",
                                'action')
                            self._vehicle_bypass_active = True
                        desired_offset = vehicle_bypass_offset
                        affected_by_vehicle = False
                        construction_phase = 'vehicle_bypass'
            else:
                self._stalled_vehicle_id = None
                self._stalled_vehicle_block_start = None

            if vehicle_bypass_offset is None:
                if self._vehicle_bypass_active:
                    self._display.log("Lead vehicle cleared — resuming normal lane", 'info')
                    self._vehicle_bypass_active = False
                desired_offset = 0.0

        if affected_by_vehicle and not failure_behavior:
            hazard_detected = True
            new_hazard_sources.add('vehicle')

        # Show the obstacles the ego takes into account in the VR dashboard (AR-HUD), in place of debug
        # bounding boxes in the world, flagged as dangers when the ego reacts to them. The dashboard
        # removes the ones no longer listed. With the failing behavior, vehicles and pedestrians aren't
        # taken into account (and construction zones are not even detected): they aren't shown
        vr_detections = []
        if not failure_behavior:
            if obstacle_vehicle is not None:
                vr_detections.append((obstacle_vehicle, 'vehicle', 'vehicle' in new_hazard_sources))
            for vehicle, is_danger in self._oncoming_watch.values():
                vr_detections.append((vehicle, 'vehicle', is_danger))
            if obstacle_pedestrian is not None:
                vr_detections.append((obstacle_pedestrian, 'pedestrian', 'pedestrian' in new_hazard_sources))
        if obstacle_tlight is not None:
            vr_detections.append((obstacle_tlight, 'traffic_light', 'traffic_light' in new_hazard_sources))
        for prop, needs_lane_change in self._construction_watch:
            vr_detections.append((prop, 'construction', needs_lane_change or 'construction' in new_hazard_sources))
        if obstacle_prop is not None:
            vr_detections.append((obstacle_prop, 'construction', True))
        # Parked cars near the ego ('full' mode): seen, not dangers (unless avoided, listed above)
        # (spawned by the leaderboard as props on some maps, part of the level on others, e.g. Town15)
        if self._vr_dashboard.mode == 'full':
            for prop in actors.filter('static.prop.mesh'):
                if (prop.get_location().distance(ego_location) < self.PARKED_BOX_DISTANCE
                        and is_parked_vehicle(prop)):
                    vr_detections.append((prop, 'parked', False))
            for car in self._level_parked_cars:
                if car.location.distance(ego_location) < self.PARKED_BOX_DISTANCE:
                    vr_detections.append((car, 'parked', False))
            # The vehicles and pedestrians of the route's scenarios (DynamicObjectCrossing's parked blocker and
            # crossing walker / bike, ...: role 'scenario...'), as soon as they are near, even before the ego
            # reacts to them (then dangers, listed above)
            for category, pattern in (('vehicle', 'vehicle.*'), ('pedestrian', 'walker.pedestrian.*')):
                for actor in actors.filter(pattern):
                    if (actor.attributes.get('role_name', '').startswith('scenario')
                            and actor.get_location().distance(ego_location) < self.SCENARIO_BOX_DISTANCE):
                        vr_detections.append((actor, category, False))
        # The sign announcing a construction zone (placed ahead of it) isn't highlighted
        vr_detections = [d for d in vr_detections if d[0].type_id != 'static.prop.warningconstruction']
        self._vr_dashboard.sync_obstacles(vr_detections, ego_location)

        # Log newly triggered / cleared hazards (event log — transitions only)
        _HAZARD_MSGS = {
            'vehicle':      ("Vehicle obstacle ahead — emergency stop",          'hazard'),
            'traffic_light':("Red traffic light — holding position",             'hazard'),
            'pedestrian':   ("Pedestrian in path — emergency stop",              'hazard'),
            'construction': ("Construction zone — waiting for oncoming traffic", 'hazard'),
            'stop_sign':    ("Stop sign — stopping",                             'hazard'),
        }
        for src in new_hazard_sources - self._hazard_sources:
            if src in _HAZARD_MSGS:
                self._display.log(*_HAZARD_MSGS[src])
        if not new_hazard_sources and self._hazard_sources:
            self._display.log("Path clear — resuming normal driving", 'clear')

        self._hazard_sources = new_hazard_sources

        # Update status bar
        _STATE_LABELS = {
            'vehicle':       'VEHICLE AHEAD',
            'traffic_light': 'RED LIGHT',
            'pedestrian':    'PEDESTRIAN',
            'construction':  'CONSTRUCTION',
            'stop_sign':     'STOP SIGN',
        }
        if new_hazard_sources:
            parts = [_STATE_LABELS[s] for s in new_hazard_sources if s in _STATE_LABELS]
            state_str = 'STOPPED — ' + ' + '.join(parts) if parts else 'STOPPED'
        else:
            state_str = 'DRIVING'
        self._display.update_status(speed=speed_kmh, state=state_str, hazard=hazard_detected)

        # NOW line — stable sentences, updated only when mode changes
        if new_hazard_sources:
            priority = next(
                (s for s in ('construction', 'pedestrian', 'vehicle', 'traffic_light', 'stop_sign')
                 if s in new_hazard_sources), None)
            _NOW_MSGS = {
                'vehicle':       ("Vehicle on the road — waiting for it to clear",     'hazard'),
                'traffic_light': ("Stopped at red traffic light",                      'hazard'),
                'pedestrian':    ("Pedestrian crossing — waiting",                     'hazard'),
                'construction':  ("Construction ahead — waiting for oncoming traffic", 'hazard'),
                'stop_sign':     ("Stopped at stop sign",                              'hazard'),
            }
            now_msg, now_lvl = _NOW_MSGS.get(priority, ("Stopped — hazard detected", 'hazard'))
        elif construction_offset is not None:
            now_msg, now_lvl = "Bypassing construction zone", 'action'
        else:
            now_msg, now_lvl = "Following path and stopping for traffic lights", 'clear'

        if now_msg != self._last_now_msg:
            self._display.update_current(now_msg, now_lvl)
            self._last_now_msg = now_msg

        # Slide the lateral offset towards its target along a smooth S-curve. The curve is defined over
        # the distance driven (not time): its slope (sideways meters per meter driven) grows and shrinks
        # gradually, so the path has the same gentle shape at any speed, and the offset doesn't move
        # while the ego is stopped (which would make it steer hard when moving off)
        dt = self._world.get_snapshot().timestamp.delta_seconds or self._dt
        driven = vehicle_speed * dt
        if desired_offset is not None:
            max_slope = self._offset_tight_slope if self._tight_side_shift else self._offset_max_slope
            slope_rate = self._offset_tight_slope_rate if self._tight_side_shift else self._offset_slope_rate
            error = desired_offset - self._offset
            # Steepest slope that can still flatten out exactly at the target
            wanted_slope = math.copysign(min(max_slope, math.sqrt(2.0 * slope_rate * abs(error))), error)
            max_change = slope_rate * driven
            self._offset_slope += max(-max_change, min(max_change, wanted_slope - self._offset_slope))
            if abs(error) < 0.01 and abs(self._offset_slope) < 0.005:
                self._offset_slope = 0.0
                self._tight_side_shift = False  # shift done
                self.set_offset(desired_offset)
            else:
                self.set_offset(self._offset + self._offset_slope * driven)

        # Lane changes of the route itself: blend into the new lane over some distance instead of
        # steering towards it at once
        lane_change_offset = self._lane_change_offset()
        self._local_planner.set_offset(self._offset + lane_change_offset)

        # Slow down for upcoming curves, and while bypassing (not for small nudges in the lane)
        curve_speed_limit = self._curve_speed_limit()
        target_speed = min(self._target_speed, curve_speed_limit)
        if abs(self._offset) > 0.5:
            target_speed = min(target_speed, self._bypass_speed)
        if obstacle_speed_limit is not None:
            target_speed = min(target_speed, obstacle_speed_limit)
        if stop_sign_speed_limit is not None:
            target_speed = min(target_speed, stop_sign_speed_limit)
        if light_speed_limit is not None:
            target_speed = min(target_speed, light_speed_limit)
        if abs(self._offset_slope) > 0.01:
            # Side shift in progress (going around an obstacle or getting back): keep it smooth
            target_speed = min(target_speed, self._shift_speed())
        self._local_planner.set_speed(target_speed)

        # 'text' visualization: a sentence telling what the ego is doing and why
        if self._vr_dashboard.mode == 'text':
            self._narrate(
                speed_kmh=speed_kmh, ego_wpt=ego_wpt, hazards=new_hazard_sources,
                obstacle_vehicle=obstacle_vehicle if 'vehicle' in new_hazard_sources else None,
                obstacle_tlight=obstacle_tlight, light_speed_limit=light_speed_limit,
                stop_sign_speed_limit=stop_sign_speed_limit, construction_phase=construction_phase,
                obstacle_room=obstacle_room, obstacle_speed_limit=obstacle_speed_limit,
                curve_speed_limit=curve_speed_limit, target_speed=target_speed,
                lane_change_offset=lane_change_offset)

        control = self._local_planner.run_step()
        if hazard_detected:
            control = self.add_emergency_stop(control)

        self._vr_dashboard.update_ego(speed_kmh, control.steer)
        self._vr_dashboard.update_path(ego_location)  # 'full' mode: the global path ahead of the ego
        self._vr_dashboard.push()

        # What the agent perceived and decided this step, for the session logger (analysis afterwards)
        self.step_info = {
            'failure_behavior': bool(failure_behavior),
            'hazards': sorted(new_hazard_sources),
            'emergency_stop': bool(hazard_detected),
            'target_speed': target_speed,
            'curve_speed_limit': curve_speed_limit,
            'light_speed_limit': light_speed_limit,
            'stop_sign_speed_limit': stop_sign_speed_limit,
            'obstacle_speed_limit': obstacle_speed_limit,
            'construction_phase': construction_phase,
            'construction_room': obstacle_room,
            'lateral_offset': lateral_offset,
            'planner_offset': self._offset,
            'road_option': str(road_option).split('.')[-1],
            'vehicle_obstacle_id': obstacle_vehicle.id if obstacle_vehicle is not None else None,
            'pedestrian_obstacle_id': obstacle_pedestrian.id if obstacle_pedestrian is not None else None,
            'traffic_light_id': obstacle_tlight.id if obstacle_tlight is not None else None,
            'oncoming_watched': len(self._oncoming_watch),
            'command': (control.throttle, control.steer, control.brake),
            # What the system perceives and takes into account this step, whatever the display mode (the
            # content the 'full' mode shows as boxes, without the parked cars, which are only shown):
            # (actor id, category, is_danger)
            'detections': [(actor.id, category, bool(danger)) for actor, category, danger in vr_detections
                           if category != 'parked'],
        }

        return control

    def _upcoming_maneuver(self, max_distance=35.0):
        """
        Next maneuver of the route within max_distance meters: ('left' / 'right' / 'straight', distance)
        for a junction, ('lane_left' / 'lane_right', distance) for a lane change, or (None, None)
        """
        ego_location = self._vehicle.get_location()
        previous = ego_location
        distance = 0.0
        for wp, option in self._local_planner.get_plan():
            location = wp.transform.location
            distance += previous.distance(location)
            previous = location
            if distance > max_distance:
                break
            if option == RoadOption.CHANGELANELEFT:
                return 'lane_left', distance
            if option == RoadOption.CHANGELANERIGHT:
                return 'lane_right', distance
            if wp.is_junction and option in (RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT):
                return {RoadOption.LEFT: 'left', RoadOption.RIGHT: 'right',
                        RoadOption.STRAIGHT: 'straight'}[option], distance
        return None, None

    def _narrate(self, speed_kmh, ego_wpt, hazards, obstacle_vehicle, obstacle_tlight, light_speed_limit,
                 stop_sign_speed_limit, construction_phase, obstacle_room, obstacle_speed_limit,
                 curve_speed_limit, target_speed, lane_change_offset):
        """
        'text' visualization: shows in the VR dashboard one sentence telling what the ego is doing
        and why, built from what actually drives it this tick (what it brakes / stops for, the
        maneuver in progress, the speed limit it follows), most important first:
        stop / braking reasons > roadworks > lane change > junction > curve > cruising.
        Moving off after a stop is announced for a few seconds. A new braking reason is urgent
        (replaces the current sentence almost at once).
        """
        now = self._world.get_snapshot().timestamp.elapsed_seconds
        stopped = speed_kmh < 1.0
        key, sentence = None, None

        # --- What the ego stops / brakes for
        if 'pedestrian' in hazards:
            key = 'pedestrian'
            sentence = ("Waiting for the pedestrian to cross" if stopped
                        else "Pedestrian on the road ahead: braking")
        elif 'vehicle' in hazards and obstacle_vehicle is not None:
            oncoming = self._oncoming_watch.get(obstacle_vehicle.id)
            target_wpt = self._map.get_waypoint(obstacle_vehicle.get_location())
            ego_fwd = self._vehicle.get_transform().get_forward_vector()
            target_fwd = obstacle_vehicle.get_transform().get_forward_vector()
            alignment = ego_fwd.x * target_fwd.x + ego_fwd.y * target_fwd.y
            target_stopped = get_speed(obstacle_vehicle) < 1.0
            if oncoming is not None and oncoming[1]:
                key = 'vehicle_oncoming'
                sentence = ("Waiting for a gap in the oncoming traffic to turn left" if stopped
                            else "Oncoming car: slowing down before turning left")
            elif (ego_wpt.is_junction or target_wpt.is_junction) and alignment < 0.5:
                key = 'vehicle_crossing'
                sentence = ("Waiting for the car crossing the junction" if stopped
                            else "Car crossing the junction: giving way")
            elif alignment > 0.5:
                key = 'vehicle_lead'
                if target_stopped:
                    sentence = ("Stopped behind a car standing ahead" if stopped
                                else "Car stopped ahead: braking to stop behind it")
                else:
                    sentence = ("Waiting behind the car ahead" if stopped
                                else "Car ahead is slower: slowing down to keep a safe distance")
            else:
                key = 'vehicle_other'
                sentence = ("Waiting for the car in the way to move" if stopped
                            else "Car in the way: braking")
        elif obstacle_tlight is not None:
            key = 'traffic_light'
            if obstacle_tlight.state == carla.TrafficLightState.Yellow and not stopped:
                sentence = "Light turning yellow: stopping before the line"
            elif stopped:
                sentence = "Waiting at the red light"
            else:
                sentence = "Red light ahead: slowing down to stop at the line"
        elif 'stop_sign' in hazards or stop_sign_speed_limit is not None:
            key = 'stop_sign'
            sentence = ("Stopped at the stop sign: checking the junction before going" if stopped
                        else "Stop sign ahead: slowing down to stop at the line")
        elif construction_phase in ('wait', 'blocked'):
            key = 'construction_wait'
            if construction_phase == 'blocked':
                sentence = "Road blocked by roadworks: stopping"
            elif stopped:
                sentence = "Roadworks ahead: waiting for the oncoming car to pass before going around them"
            else:
                sentence = "Roadworks ahead with oncoming traffic: slowing down to let it pass"

        # --- Moving off after a stop
        if key is None and self._narration_key in (
                'pedestrian', 'vehicle_oncoming', 'vehicle_crossing', 'vehicle_lead', 'vehicle_other',
                'traffic_light', 'stop_sign', 'construction_wait') and self._narration_was_stopped:
            reason = {
                'pedestrian': "The pedestrian has crossed",
                'vehicle_oncoming': "Gap in the oncoming traffic",
                'vehicle_crossing': "The junction is clear",
                'vehicle_lead': "The car ahead moved on",
                'vehicle_other': "The way is clear",
                'traffic_light': "The light is green",
                'stop_sign': "The junction is clear",
                'construction_wait': "The oncoming lane is clear",
            }[self._narration_key]
            self._narration_resume = ("{}: moving off".format(reason), now + 3.0)
        if key is None and self._narration_resume is not None:
            if now < self._narration_resume[1]:
                key, sentence = 'resume', self._narration_resume[0]
            else:
                self._narration_resume = None
        if key is not None and key != 'resume':
            self._narration_resume = None

        # --- Maneuvers
        if key is None and construction_phase is not None:
            key = 'construction_' + construction_phase
            if construction_phase == 'approach':
                sentence = ("Roadworks ahead: slowing down to go around them" if obstacle_speed_limit is not None
                            else "Roadworks ahead: preparing to go around them")
            elif construction_phase == 'bypass':
                shifting_out = (abs(self._offset_slope) > 0.01 and self._bypass_target is not None
                                and abs(self._bypass_target - self._offset) > 0.3)
                sentence = ("Moving into the opposite lane to pass the roadworks" if shifting_out
                            else "Passing the roadworks in the opposite lane")
            elif construction_phase == 'nudge':
                sentence = "Obstacle on the edge of the lane: moving slightly aside"
            elif construction_phase == 'vehicle_bypass':
                sentence = "Car stopped in our lane: going around it"
        if key is None and abs(self._offset) > 0.3:
            key = 'return'
            sentence = "Obstacle passed: returning to our lane"

        if key is None:
            maneuver, distance = self._upcoming_maneuver()
            if maneuver in ('lane_left', 'lane_right'):
                self._last_lane_change_dir = 'left' if maneuver == 'lane_left' else 'right'
            if abs(lane_change_offset) > 0.2 or maneuver in ('lane_left', 'lane_right'):
                direction = self._last_lane_change_dir or ('left' if lane_change_offset > 0 else 'right')
                key = 'lane_change'
                sentence = "Changing to the {} lane to follow the route".format(direction)
            elif ego_wpt.is_junction and self._local_planner.target_road_option in (
                    RoadOption.LEFT, RoadOption.RIGHT, RoadOption.STRAIGHT):
                option = self._local_planner.target_road_option
                key = 'junction'
                sentence = {RoadOption.LEFT: "Turning left at the junction",
                            RoadOption.RIGHT: "Turning right at the junction",
                            RoadOption.STRAIGHT: "Going straight through the junction"}[option]
            elif maneuver in ('left', 'right', 'straight') and distance < 30.0:
                key = 'junction_ahead'
                sentence = {'left': "Junction ahead: getting ready to turn left",
                            'right': "Junction ahead: getting ready to turn right",
                            'straight': "Junction ahead: going straight on"}[maneuver]
            elif curve_speed_limit < self._target_speed - 3.0 and target_speed <= curve_speed_limit + 0.5:
                key = 'curve'
                sentence = "Curve ahead: slowing down to {:.0f} km/h".format(curve_speed_limit)
            else:
                key = 'cruise'
                sentence = "Road clear: driving at {:.0f} km/h".format(round(target_speed / 5.0) * 5.0)

        # A new reason to brake is urgent: shown almost at once
        urgent = key != self._narration_key and key in (
            'pedestrian', 'vehicle_oncoming', 'vehicle_crossing', 'vehicle_lead', 'vehicle_other',
            'traffic_light', 'stop_sign', 'construction_wait')
        if key != self._narration_key:
            self._narration_key = key
            self._narration_since = now
            self._narration_was_stopped = False
        self._narration_was_stopped = self._narration_was_stopped or stopped
        self._vr_dashboard.set_text(sentence, urgent=urgent)

    def _lane_change_offset(self):
        """
        Extra lateral offset (positive to the right) of the steering target that turns the sideways
        jumps of the plan (lane changes, see _find_lane_changes) into smooth S-curves. The ego's
        sideways position follows a 'smootherstep' profile over _lane_change_length meters, centered
        on the jump: zero sideways speed and acceleration at both ends, no sudden turn of the wheel.
        The offset is computed for the target waypoint (the point the ego steers to), from its
        distance along the plan, so the curve is continuous on both sides of the jump.
        """
        if not self._lane_changes:
            return 0.0
        queue_length = len(self._local_planner.get_plan())
        index = len(self._plan_s) - queue_length
        if not 0 <= index < len(self._plan_s):
            return 0.0
        return self._lane_change_offset_at(self._plan_s[index])

    def _smoothed_plan(self, plan):
        """The plan with its lane changes smoothed (see _lane_change_offset): [(point, road option)], each
        point having the shifted .transform and its waypoint's .lane_width"""
        smoothed = []
        for index, (wp, option) in enumerate(plan):
            transform = wp.transform
            offset = self._lane_change_offset_at(self._plan_s[index]) if index < len(self._plan_s) else 0.0
            if abs(offset) > 1e-3:
                r_vec = transform.get_right_vector()
                transform = carla.Transform(transform.location + carla.Location(x=offset * r_vec.x, y=offset * r_vec.y),
                                            transform.rotation)
            smoothed.append((types.SimpleNamespace(transform=transform, lane_width=wp.lane_width), option))
        return smoothed

    def _lane_change_offset_at(self, s_target):
        """Lateral offset (positive to the right) of the smoothed path from the plan's waypoint at s_target
        meters along the plan (see _lane_change_offset)"""
        offset = 0.0
        for start, end, group in self._lane_changes:
            if not start < s_target < end:
                continue
            p = (s_target - start) / (end - start)
            smooth = p * p * p * (p * (6.0 * p - 15.0) + 10.0)
            total = sum(sideways for _, sideways in group)
            # Lane of the plan at the target, relative to the lane before the group of jumps
            plan_lateral = sum(sideways for s_jump, sideways in group if s_target >= s_jump)
            # Desired position relative to that same lane: a smooth S-curve to the final lane
            offset += total * smooth - plan_lateral
        return offset

    def _road_edges(self, wp):
        """
        Distances (m) from the center of wp's lane to the outer edge of the drivable
        road on its right and on its left, following Driving lanes on both sides
        (including the opposite-direction ones) plus the paved shoulder beyond them.
        """
        edges = []
        for side in (1, -1):
            dist = wp.lane_width / 2.0
            current, direction = wp, side
            for _ in range(10):
                nxt = current.get_right_lane() if direction > 0 else current.get_left_lane()
                if nxt is None or nxt.lane_type not in (carla.LaneType.Driving, carla.LaneType.Shoulder):
                    break
                if nxt.lane_id * current.lane_id < 0:
                    direction = -direction  # crossed the center line, lane sides flip
                dist += nxt.lane_width
                current = nxt
                if nxt.lane_type == carla.LaneType.Shoulder:
                    break
            edges.append(dist)
        return edges[0], edges[1]

    # [m] parked cars closer than this to the ego get a box in the VR dashboard ('full' mode)
    PARKED_BOX_DISTANCE = 40.0
    # [m] the scenarios' vehicles and pedestrians closer than this get a box ('full' mode)
    SCENARIO_BOX_DISTANCE = 60.0

    # Steps a prop's obstacle description is reused before being computed again (props are static, but
    # may be pushed: ~1 s)
    PROP_OBSTACLE_REFRESH = 20

    def _prop_obstacles(self, props):
        """
        Static obstacle descriptions of the prop actors, cached by actor id (there are hundreds of props
        with the parked cars: computing them all at every step slowed the steps down)
        """
        self._prop_obstacle_step += 1
        step, cache = self._prop_obstacle_step, self._prop_obstacle_cache
        obstacles, alive = [], set()
        for prop in props:
            alive.add(prop.id)
            cached = cache.get(prop.id)
            # Each prop is refreshed at its own step (by id), not all of them at the same step
            if cached is None or (step - prop.id) % self.PROP_OBSTACLE_REFRESH == 0:
                cached = cache[prop.id] = (step, self._prop_obstacle(prop))
            obstacles.append(cached[1])
        if len(cache) > len(alive):
            for prop_id in [i for i in cache if i not in alive]:
                del cache[prop_id]
        return obstacles

    @staticmethod
    def _prop_obstacle(prop):
        """Static obstacle description (center, extent, rotation, actor) of a prop actor"""
        transform = prop.get_transform()
        center = transform.transform(prop.bounding_box.location)
        return (carla.Location(x=center.x, y=center.y, z=center.z),
                prop.bounding_box.extent, transform.rotation, prop)

    def _find_level_obstacles(self, plan, max_distance=8.0):
        """
        Small fixed objects that are part of the level itself (bollards, sign and light
        poles, posts, ...) close to the route. They aren't actors, so they are invisible
        to the actor-based checks, and in some junctions they stand right on the edge of
        the lane given by the map. Computed once per route, as static obstacle
        descriptions (center, extent, rotation, None).
        """
        labels = [carla.CityObjectLabel.Static, carla.CityObjectLabel.Poles,
                  carla.CityObjectLabel.TrafficSigns, carla.CityObjectLabel.Dynamic,
                  carla.CityObjectLabel.Other]
        cell = max_distance
        grid = {}
        for wp, _ in plan:
            loc = wp.transform.location
            grid.setdefault((int(loc.x // cell), int(loc.y // cell)), []).append(loc)

        obstacles = []
        for label in labels:
            for obj in self._world.get_environment_objects(label):
                box = obj.bounding_box
                extent = box.extent
                # Only compact objects: long ones (guard rails, fences, walls) line the road
                # side and their boxes would swallow the lane
                if max(extent.x, extent.y) > 1.5 or extent.z < 0.1:
                    continue
                center = box.location
                key = (int(center.x // cell), int(center.y // cell))
                close = False
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for loc in grid.get((key[0] + dx, key[1] + dy), ()):
                            if (loc.distance(center) < max_distance
                                    and center.z - extent.z < loc.z + 2.0
                                    and center.z + extent.z > loc.z + 0.1):
                                close = True
                                break
                        if close:
                            break
                    if close:
                        break
                if close:
                    obstacles.append((carla.Location(x=center.x, y=center.y, z=center.z),
                                      extent, box.rotation, None))
        return obstacles

    def _find_parked_cars(self, plan, max_distance=15.0):
        """
        Parked cars that are part of the level itself (Town15: the leaderboard spawns no parked cars
        there), close to the route: for their boxes in the VR dashboard. Computed once per route, as
        LevelObject's. Objects overlapping a vehicle actor at that time are left out (actors are shown
        as such)
        """
        vehicle_labels = (carla.CityObjectLabel.Car, carla.CityObjectLabel.Truck, carla.CityObjectLabel.Bus)
        cell = max_distance
        grid = set()
        for wp, _ in plan:
            loc = wp.transform.location
            grid.add((int(loc.x // cell), int(loc.y // cell)))
        actor_locations = [a.get_location() for a in self._world.get_actors().filter('*vehicle*')]

        candidates = []  # (world bounding box, is a truck)
        try:
            objects = self._world.get_environment_objects(carla.CityObjectLabel.Any)
        except RuntimeError:
            return []
        for obj in objects:
            name = obj.name.lower()
            if obj.type not in vehicle_labels and 'parked' not in name:
                continue
            box = obj.bounding_box
            if not 1.5 < box.extent.x < 8.0 or box.extent.y < 0.6:
                continue  # not car-sized (also: a whole group of cars merged in one mesh)
            center = box.location
            key = (int(center.x // cell), int(center.y // cell))
            if not any((key[0] + dx, key[1] + dy) in grid for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
                continue
            if any(loc.distance(center) < 2.0 for loc in actor_locations):
                continue
            truck = obj.type in (carla.CityObjectLabel.Truck, carla.CityObjectLabel.Bus) or box.extent.x > 3.5
            candidates.append((box, truck))

        # One car is often several objects (its body, windows, interior... each its own mesh with the car's
        # tag): the objects whose centers are close (closer than half the car's width) are one car, kept
        # with its biggest box
        candidates.sort(key=lambda c: c[0].extent.x * c[0].extent.y * c[0].extent.z, reverse=True)
        kept = []
        for box, truck in candidates:
            center = box.location
            if any(math.hypot(center.x - k.location.x, center.y - k.location.y) < max(k.extent.y, 1.0)
                   for k, _ in kept):
                continue
            kept.append((box, truck))
        parked = [LevelObject(PARKED_CAR_FIRST_ID + i, box, truck=truck) for i, (box, truck) in enumerate(kept)]
        print("VR dashboard: {} parked cars of the level along the route ({} objects)".format(
            len(parked), len(candidates)))
        return parked

    def _plan_frame(self, ego_location, lookahead):
        """
        The plan ahead as (distance along it from the ego, waypoint), starting level with the
        ego. Used to express locations in route coordinates (see _project_on_plan).
        """
        plan = []
        dist = 0.0
        prev_location = ego_location
        for wp, _ in self._local_planner.get_plan():
            location = wp.transform.location
            dist += location.distance(prev_location)
            prev_location = location
            if dist > lookahead:
                break
            plan.append((dist, wp))
        if not plan:
            return plan
        # The planner has already dropped the few meters around the ego: add the route's
        # lane waypoint level with the ego (not the ego's current lane, which is the
        # opposite one while bypassing), so locations alongside are measured correctly
        behind = plan[0][1].previous(max(plan[0][0], 0.5))
        if behind:
            plan.insert(0, (0.0, behind[0]))
        return plan

    @staticmethod
    def _project_on_plan(plan, location):
        """
        Route coordinates of a location: (s, d, waypoint) with s the distance along the plan
        from the ego and d the signed lateral distance to the route's lane center (positive
        to the right). None if the location isn't beside the plan.
        """
        s_wp, wp = min(plan, key=lambda p: p[1].transform.location.distance(location))
        vec = location - wp.transform.location
        f_vec = wp.transform.get_forward_vector()
        r_vec = wp.transform.get_right_vector()
        along = vec.x * f_vec.x + vec.y * f_vec.y
        if abs(along) > 3.0:
            return None  # before the plan's start or past its end
        return s_wp + along, vec.x * r_vec.x + vec.y * r_vec.y, wp

    def _static_obstacle_avoidance(self, ego_location, vehicle_speed, props):
        """
        Finds the static obstacles intruding the ego's corridor along the plan and the
        smallest lateral offset (positive to the right) that clears all of them while
        staying on the drivable road.

            :param props: static obstacle descriptions (center, extent, rotation, actor or None)
            :return: (offset, prop, room, nudge_offset, needs_lane_change).
                - offset: offset clearing all the obstacles in the way. None when no avoidance is
                  needed (then room is None too), or when it's impossible.
                - room: distance from the ego's front to the nearest obstacle that needs a lane change
                  (or to the nearest obstacle in the way, if all of them can be passed in the lane).
                - nudge_offset: offset clearing only the obstacles that can be passed staying in the
                  lane (small nudges), used until the lane change starts.
                - needs_lane_change: whether some obstacle can't be passed staying in the lane.
        """
        half_width = self._vehicle.bounding_box.extent.y
        half_length = self._vehicle.bounding_box.extent.x
        lookahead = self._static_obstacle_lookahead
        self._construction_watch = []  # [(prop actor, whether it needs a lane change)] shown in the dashboard

        plan = self._plan_frame(ego_location, lookahead)
        if not plan:
            return None, None, None, 0.0, False

        obstacles = []  # (s along the plan, signed lateral distance d, radius, waypoint, actor or None)
        for location, extent, rotation, prop in props:
            if extent.z < 0.1:
                continue  # flat (debris, decals): can be driven over
            if location.distance(ego_location) > lookahead + 5.0:
                continue
            projection = self._project_on_plan(plan, location)
            if projection is None:
                continue
            s, d, wp = projection
            r_vec = wp.transform.get_right_vector()
            # Keep obstacles until they're well behind the ego's rear, so the offset holds while passing
            # and the ego only starts getting back to its lane once clearly past them
            if s < -(half_length + self._obstacle_return_gap) or s > lookahead or abs(d) > 10.0:
                continue
            # Half-width of the prop across the route, from its actual orientation. Never
            # less than its largest half-size: some props have degenerate bounding boxes
            # (e.g. the construction cone's is 0.35 x 0.04 m)
            p_fwd = rotation.get_forward_vector()
            p_right = rotation.get_right_vector()
            lateral_extent = max(extent.x * abs(p_fwd.x * r_vec.x + p_fwd.y * r_vec.y)
                                 + extent.y * abs(p_right.x * r_vec.x + p_right.y * r_vec.y),
                                 extent.x, extent.y)
            obstacles.append((s, d, lateral_extent, wp, prop))

        def conflicts(offset, margin):
            # The small tolerance stops an offset computed to sit exactly at an obstacle's
            # margin from being flagged again by that same obstacle (float rounding)
            return [o for o in obstacles if abs(o[1] - offset) < o[2] + half_width + margin - 1e-3]

        blocking = conflicts(0.0, self._static_obstacle_margin)
        if not blocking:
            return None, None, None, 0.0, False

        def clearing_offset(subset, ignore=()):
            """
            Smallest offset passing all the obstacles of 'subset' on the left or on the right (and any
            other obstacle met there, except the 'ignore' ones), with the preferred margin first and a
            tighter one if the road is too narrow for it
            """
            edges = [self._road_edges(o[3]) for o in subset]
            candidates = []
            for margin in (self._static_obstacle_margin, self._static_obstacle_min_margin):
                for side in (-1, 1):
                    offset, hits = 0.0, subset
                    for _ in range(5):
                        if side < 0:
                            offset = min(o[1] - o[2] for o in hits) - half_width - margin
                        else:
                            offset = max(o[1] + o[2] for o in hits) + half_width + margin
                        hits = [o for o in conflicts(offset, margin) if not any(o is x for x in ignore)]
                        if not hits:
                            break
                    if hits:
                        continue
                    # Stay on the road next to every obstacle being passed
                    if all(offset + half_width <= right_edge and offset - half_width >= -left_edge
                           for right_edge, left_edge in edges):
                        candidates.append(offset)
                if candidates:
                    break
            if not candidates:
                return None
            return min(candidates, key=lambda o: (abs(o), o > 0))  # smallest shift, left on ties

        # Obstacles that can't be passed staying in the lane (e.g. the construction panel and cones)
        # vs those only needing a small nudge (e.g. the warning sign on the side of the road)
        def needs_lane_change(o):
            in_lane = max(0.0, o[3].lane_width / 2.0 - half_width)
            reach = o[2] + half_width + self._static_obstacle_min_margin
            return min(abs(o[1] - reach), abs(o[1] + reach)) > in_lane
        major = [o for o in blocking if needs_lane_change(o)]
        nearest = min(major or blocking, key=lambda o: o[0])
        # Nudges only for the obstacles met before the lane change (the others are passed by it)
        minor = [o for o in blocking if not any(o is x for x in major) and (not major or o[0] < nearest[0])]

        room = nearest[0] - half_length - nearest[2]

        # Props of the zone being handled (cones, warning panel and signs on or next to the lane),
        # shown in the dashboard: the ones forcing the lane change are flagged as dangers
        for o in obstacles:
            if o[4] is None or abs(o[1]) > 1.5 * o[3].lane_width:
                continue
            self._construction_watch.append((o[4], any(o is x for x in major)))

        beyond = [o for o in obstacles if major and o[0] >= nearest[0]]
        nudge_offset = (clearing_offset(minor, ignore=major + beyond) if minor else 0.0) or 0.0
        offset = clearing_offset(blocking)
        return offset, nearest[4], room, nudge_offset, bool(major)

    def _stop_sign_control(self, ego_wpt, vehicle_speed):
        """
        Makes the ego stop briefly at the stop signs of its route: it brakes smoothly to stop just
        before the sign, stands still for _stop_sign_wait seconds, then goes on (and doesn't stop
        again for that sign). Stop signs are OpenDRIVE landmarks of type 206 that apply to the ego's
        lane, ahead of it on its route.

            :return: (speed limit [km/h] or None, whether to stop now)
        """
        if self._ignore_stop_signs:
            return None, False

        landmarks = ego_wpt.get_landmarks_of_type(self._stop_sign_lookahead, '206', False)
        on_route = [lm for lm in landmarks if self._landmark_on_route(lm, ego_wpt)]
        # Forget the signs already stopped at once they are no longer ahead
        self._stop_signs_done &= {lm.id for lm in on_route}
        pending = [lm for lm in on_route if lm.id not in self._stop_signs_done]
        if not pending:
            self._stop_sign_state = None
            return None, False

        sign = min(pending, key=lambda lm: lm.distance)
        if self._stop_sign_state is None or self._stop_sign_state[0] != sign.id:
            self._stop_sign_state = [sign.id, None]
            self._display.log(f"Stop sign ahead — stopping in {sign.distance:.0f} m", 'info')

        remaining = sign.distance - self._vehicle.bounding_box.extent.x - self._stop_sign_gap
        if remaining > 0.5 and self._stop_sign_state[1] is None:
            # Approaching: brake smoothly to reach the sign at a crawl
            speed = math.sqrt(2.0 * self._comfort_decel * remaining) * 3.6
            return max(speed, self._stop_sign_creep_speed), False

        # At the sign: stop, wait, then go on
        now = self._world.get_snapshot().timestamp.elapsed_seconds
        if self._stop_sign_state[1] is None and vehicle_speed < 0.1:
            self._stop_sign_state[1] = now
        if self._stop_sign_state[1] is not None and now - self._stop_sign_state[1] >= self._stop_sign_wait:
            self._stop_signs_done.add(sign.id)
            self._stop_sign_state = None
            self._display.log("Stop sign — going on", 'info')
            return None, False
        return 0.0, True

    def _landmark_on_route(self, landmark, ego_wpt, s_tolerance=8.0):
        """Whether a landmark found ahead along the ego's lane is on the road the plan follows"""
        if landmark.road_id == ego_wpt.road_id:
            return True
        dist = 0.0
        prev = ego_wpt.transform.location
        for wp, _ in self._local_planner.get_plan():
            dist += wp.transform.location.distance(prev)
            prev = wp.transform.location
            if dist > self._stop_sign_lookahead + 10.0:
                break
            if wp.road_id == landmark.road_id and abs(wp.s - landmark.s) < s_tolerance:
                return True
        return False

    def _shift_speed(self):
        """Speed [km/h] at which the (gentle) side shift stays under its maximum lateral acceleration"""
        return math.sqrt(self._shift_max_lateral_accel / self._offset_slope_rate) * 3.6

    def _shift_distance(self, shift, tight):
        """Distance [m] needed to move 'shift' meters sideways with the (gentle or tight) side shift profile"""
        max_slope = self._offset_tight_slope if tight else self._offset_max_slope
        slope_rate = self._offset_tight_slope_rate if tight else self._offset_slope_rate
        return shift / max_slope + max_slope / slope_rate

    def _curve_speed_limit(self):
        """
        Returns the highest speed (km/h) that keeps the lateral acceleration in the
        upcoming curves under _max_lateral_accel, while leaving room to brake comfortably
        before reaching them. Curvature is estimated from the route positions (not the
        waypoint yaws, which can jump between overlapping lanes inside junctions).
        """
        # Resample the plan ahead at roughly fixed spacing. Start from the route level with the
        # ego: the planner has already dropped the waypoints within a few meters of the ego, so
        # without it a curve the ego is already driving through would be missed. Not from the ego
        # itself, as it's off the route while driving around an obstacle: that sideways gap would
        # look like a sharp bend, making the ego crawl along the opposite lane.
        ego_location = self._vehicle.get_location()
        route_level = self._plan_frame(ego_location, 5.0)
        prev_location = route_level[0][1].transform.location if route_level else ego_location
        points = [(0.0, prev_location)]  # (distance along the plan, location)
        dist = 0.0
        last_sample_dist = 0.0
        for wp, _ in self._local_planner.get_plan():
            location = wp.transform.location
            dist += location.distance(prev_location)
            prev_location = location
            if dist > self._curve_lookahead:
                break
            if dist - last_sample_dist >= self._curve_sample_step:
                points.append((dist, location))
                last_sample_dist = dist

        speed_limit = self._target_speed
        for i in range(len(points) - 2):
            (d_a, a), (_, b), (_, c) = points[i], points[i + 1], points[i + 2]
            ab, bc, ca = a.distance(b), b.distance(c), c.distance(a)
            cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
            if abs(cross) < 1e-3:
                continue  # straight
            radius = ab * bc * ca / (2.0 * abs(cross))
            curve_speed = math.sqrt(self._max_lateral_accel * radius)  # m/s
            allowed_now = math.sqrt(curve_speed ** 2 + 2.0 * self._comfort_decel * d_a) * 3.6
            speed_limit = min(speed_limit, allowed_now)

        # Also respect the curvature the steering controller is currently asking for
        lateral_controller = self._local_planner._vehicle_controller._lat_controller
        curvature = abs(getattr(lateral_controller, 'last_curvature', 0.0))
        if curvature > 1e-3:
            speed_limit = min(speed_limit, math.sqrt(self._max_lateral_accel / curvature) * 3.6)

        return max(self._min_curve_speed, speed_limit)

    def _pedestrian_obstacle_detected(self, pedestrian_list, max_distance):
        """
        Method to check if there are pedestrians in the ego's path ahead, or walking into it.

        A pedestrian merely standing or walking near the road (on the sidewalk, or on the
        shoulder where there's none) isn't a hazard. Braking for it anyway is a livelock: it
        drops the ego's speed to ~0, which makes InTimeToArrivalToLocation's ETA blow up, so
        the crossing scenario's own trigger never fires and the pedestrian never crosses.
        Walker speeds can't be read from the server (they're moved kinematically, reporting
        no velocity), so they're estimated from position changes between ticks.

            :param pedestrian_list (list of carla.Walker): list containing Walker objects.
            :param max_distance (float): max distance for pedestrians to be considered relevant.
        """

        if self._ignore_pedestrians:
            return False, None

        ego_location = self._vehicle.get_location()
        now = self._world.get_snapshot().timestamp.elapsed_seconds
        corridor = self._vehicle.bounding_box.extent.y + self._pedestrian_margin
        plan = None
        hazard = None
        seen = {}
        for pedestrian in pedestrian_list:
            location = pedestrian.get_location()
            previous = self._ped_last_seen.get(pedestrian.id)
            seen[pedestrian.id] = (location, now)
            if hazard is not None or location.distance(ego_location) >= max_distance:
                continue

            if plan is None:
                plan = self._plan_frame(ego_location, max_distance + 5.0)
            projection = self._project_on_plan(plan, location) if plan else None
            if projection is None:
                continue
            s, d, wp = projection
            if s < 0:
                continue  # already beside or behind the ego

            gap = d - self._offset  # lateral distance to the middle of the ego's path
            if abs(gap) < corridor:
                hazard = pedestrian
                continue

            if previous is not None and now > previous[1]:
                r_vec = wp.transform.get_right_vector()
                lateral_speed = ((location.x - previous[0].x) * r_vec.x
                                 + (location.y - previous[0].y) * r_vec.y) / (now - previous[1])
                approach_speed = -lateral_speed if gap > 0 else lateral_speed
                if approach_speed > 0.3 and abs(gap) - corridor < approach_speed * self._pedestrian_horizon:
                    hazard = pedestrian

        self._ped_last_seen = seen
        return hazard is not None, hazard

    def ignore_traffic_lights(self, active=True):
        """(De)activates the checks for traffic lights"""
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_vehicles = active

    def set_offset(self, offset):
        """Sets an offset for the vehicle"""
        # Keep self._offset in sync too - _vehicle_obstacle_detected's route polygon and
        # opposite-invasion checks read it, and previously never saw a real bypass offset
        # (it stayed at its initial value, e.g. 0, since this method only ever forwarded
        # the value to the local planner without updating it here).
        self._offset = offset
        self._local_planner.set_offset(offset)

    def lane_change(self, direction, same_lane_time=0, other_lane_time=0, lane_change_time=2):
        """
        Changes the path so that the vehicle performs a lane change.
        Use 'direction' to specify either a 'left' or 'right' lane change,
        and the other 3 fine tune the maneuver
        """
        speed = self._vehicle.get_velocity().length()
        path = self._generate_lane_change_path(
            self._map.get_waypoint(self._vehicle.get_location()),
            direction,
            same_lane_time * speed,
            other_lane_time * speed,
            lane_change_time * speed,
            False,
            1,
            self._sampling_resolution
        )
        if not path:
            self._display.log("Lane change ignored — no valid path found", 'warning')

        self.set_global_plan(path)

    def _traffic_light_control(self, ego_location, vehicle_speed):
        """
        Makes the ego stop at red traffic lights (and at yellow ones when it can stop comfortably).
        Lights are matched to the route through their stop lines (TrafficLight.get_stop_waypoints():
        one waypoint per lane they control, at the line), compared with the route's waypoints ahead by
        road, lane and position along the road. This finds the light whatever the road pieces /
        junction layout, and early enough to brake smoothly to the line.

            :return: (speed limit [km/h] or None, whether to stop now, the traffic light or None)
        """
        if self._ignore_traffic_lights:
            return None, False, None

        # Refresh the lights regularly: in large maps, lights far from the ego may only exist later
        now = self._world.get_snapshot().timestamp.elapsed_seconds
        if self._light_stop_wps is None:
            self._light_stop_wps = []
        if self._light_refresh_time is None or now - self._light_refresh_time > self._light_refresh_period:
            self._light_refresh_time = now
            known = {light.id for light, _ in self._light_stop_wps}
            self._lights_list = self._world.get_actors().filter("*traffic_light*")
            for light in self._lights_list:
                if light.id in known:
                    continue
                try:
                    stop_wps = light.get_stop_waypoints()
                except RuntimeError:
                    stop_wps = []
                self._light_stop_wps.append((light, stop_wps))

        # Once in a junction, the ego has passed its light: never stop there (trigger volumes and stop
        # lines of the other approaches reach into it, and would stop the ego while it leaves it)
        if self._map.get_waypoint(ego_location).is_junction:
            return None, False, None

        # The plan is checked a bit further than the lookahead to see what follows each stop line
        extended_plan = self._plan_frame(ego_location, self._traffic_light_lookahead + self._stop_line_junction_dist)
        if not extended_plan:
            return None, False, None
        plan = [(s_plan, wp) for s_plan, wp in extended_plan if s_plan <= self._traffic_light_lookahead]
        self._traffic_light_diagnostics(ego_location, plan)

        def leads_into_junction(s_line):
            """A real stop line is right before a junction (not e.g. at the exit of the previous one)"""
            return any(wp.is_junction for s_plan, wp in extended_plan
                       if s_line < s_plan <= s_line + self._stop_line_junction_dist)

        # Nearest stop line of a red / yellow light on the route ahead
        nearest = None  # (distance along the route to the stop line, light)
        for light, stop_wps in self._light_stop_wps:
            state = light.state
            if state not in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow):
                continue
            for stop_wp in stop_wps:
                for s_plan, wp in plan:
                    if self._at_stop_line(wp, stop_wp):
                        if (nearest is None or s_plan < nearest[0]) and leads_into_junction(s_plan):
                            nearest = (s_plan, light)
                        break
        if nearest is None:
            # Fallback: the server tells when the ego is inside a light's trigger volume. Only used
            # if that light has a stop line just ahead of the ego, facing its direction of travel
            # (the trigger volumes of other approaches can overlap the ego's lane)
            light = self._vehicle.get_traffic_light() if self._vehicle.is_at_traffic_light() else None
            if light is not None and light.state == carla.TrafficLightState.Red:
                distance = self._stop_line_ahead(light, ego_location)
                if distance is not None:
                    nearest = (distance, light)
            if nearest is None:
                return None, False, None

        s_line, light = nearest
        remaining = s_line - self._vehicle.bounding_box.extent.x - self._traffic_light_gap
        if remaining < -1.0:
            return None, False, None  # already past the line (in the junction): go on

        if light.state == carla.TrafficLightState.Yellow:
            # Like a driver: stop only if it can be done without braking hard
            if vehicle_speed ** 2 / (2.0 * self._yellow_max_decel) > max(remaining, 0.0):
                return None, False, None

        if remaining > 0.5:
            # Brake smoothly to reach the line at a crawl
            speed = math.sqrt(2.0 * self._comfort_decel * remaining) * 3.6
            return max(speed, self._traffic_light_creep_speed), False, light
        return 0.0, True, light

    @staticmethod
    def _at_stop_line(wp, stop_wp, s_tolerance=1.5):
        """
        Whether a route waypoint is at a light's stop line. A light governs all the lanes of its approach,
        but its stop lines (from its trigger volume) sometimes only cover some of them: any lane of the
        same road and direction (same lane id sign) at the same position along the road counts.
        """
        return (wp.road_id == stop_wp.road_id and wp.lane_id * stop_wp.lane_id > 0
                and abs(wp.s - stop_wp.s) < s_tolerance)

    def _stop_line_ahead(self, light, ego_location, max_distance=10.0):
        """
        Distance from the ego to a stop line of the light that is just ahead of it (within
        max_distance, roughly in its lane) and facing its direction of travel. None if there's none.
        """
        stop_wps = next((wps for l, wps in self._light_stop_wps if l.id == light.id), [])
        ego_forward = self._vehicle.get_transform().get_forward_vector()
        best = None
        for stop_wp in stop_wps:
            line_forward = stop_wp.transform.get_forward_vector()
            if line_forward.x * ego_forward.x + line_forward.y * ego_forward.y < 0.7:
                continue  # stop line of another approach
            vec = stop_wp.transform.location - ego_location
            along = vec.x * ego_forward.x + vec.y * ego_forward.y
            sideways = abs(-vec.x * ego_forward.y + vec.y * ego_forward.x)
            if 0.0 < along < max_distance and sideways < 6.0 and (best is None or along < best):
                best = along
        return best

    def _traffic_light_diagnostics(self, ego_location, plan, distance=25.0):
        """Prints once per traffic light, when the ego gets close to it, why it is (not) taken into account"""
        for light, stop_wps in self._light_stop_wps:
            if light.id in self._light_diag_done:
                continue
            light_location = light.get_location()
            if light_location.distance(ego_location) > distance:
                continue
            self._light_diag_done.add(light.id)
            matched = any(self._at_stop_line(wp, sw) for sw in stop_wps for _, wp in plan)
            print("DIAGNOSTIC: traffic light {} at ({:.1f}, {:.1f}) state {}: {} stop lines {}, {} the route "
                  "(route lanes ahead: {})".format(
                      light.id, light_location.x, light_location.y, light.state, len(stop_wps),
                      [(sw.road_id, sw.lane_id, round(sw.s, 1)) for sw in stop_wps],
                      "ON" if matched else "NOT on",
                      sorted({(wp.road_id, wp.lane_id) for _, wp in plan})[:8]))

    def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
        """
        Method to check if there is a red light affecting the vehicle.

            :param lights_list (list of carla.TrafficLight): list containing TrafficLight objects.
                If None, all traffic lights in the scene are used
            :param max_distance (float): max distance for traffic lights to be considered relevant.
                If None, the base threshold value is used
        """
        if self._ignore_traffic_lights:
            return (False, None)

        if not lights_list:
            lights_list = self._world.get_actors().filter("*traffic_light*")

        if not max_distance:
            max_distance = self._base_tlight_threshold

        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
            else:
                return (True, self._last_traffic_light)

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            if traffic_light.id in self._lights_map:
                trigger_wp = self._lights_map[traffic_light.id]
            else:
                trigger_location = get_trafficlight_trigger_location(traffic_light)
                trigger_wp = self._map.get_waypoint(trigger_location)
                self._lights_map[traffic_light.id] = trigger_wp

            if trigger_wp.transform.location.distance(ego_vehicle_location) > max_distance:
                continue

            if trigger_wp.road_id != ego_vehicle_waypoint.road_id:
                continue

            # road_id alone doesn't distinguish the two directions of a bidirectional road -
            # lane_id's sign encodes which side/direction a lane belongs to in OpenDRIVE, so
            # a mismatch here means this light governs the opposite-direction lane, not ours.
            if (trigger_wp.lane_id > 0) != (ego_vehicle_waypoint.lane_id > 0):
                continue

            ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
            wp_dir = trigger_wp.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            if is_within_distance(trigger_wp.transform, self._vehicle.get_transform(), max_distance, [0, 90]):
                self._last_traffic_light = traffic_light
                return (True, traffic_light)

        return (False, None)

    def _draw_polygon(self, polygon, z=0.5, color=(255, 0, 0), life_time=0.2):
        """
        Draws a Shapely Polygon in CARLA debug view.

        :param polygon: Shapely Polygon to draw
        :param z: height at which to draw the lines
        :param color: RGB tuple (255, 0, 0) by default for red
        :param life_time: duration the lines stay visible
        """
        points = list(polygon.exterior.coords)

        for i in range(len(points)):
            start = carla.Location(x=points[i][0], y=points[i][1], z=z)
            end = carla.Location(x=points[(i+1) % len(points)][0], y=points[(i+1) % len(points)][1], z=z)

            self._world.debug.draw_line(
                start, end, thickness=0.1,
                color=carla.Color(r=color[0], g=color[1], b=color[2]),
                life_time=life_time
            )
    def _oncoming_blocks_bypass(self, ego_location, vehicle_list, bypass_length, lookahead=150.0):
        """
        Whether oncoming traffic prevents bypassing an obstacle through the opposite lane.
        Vehicles are located along the route (not by road id, which changes every few dozen
        meters): any vehicle in the lane to the left of the route, coming towards the ego and
        reaching the bypass stretch ('bypass_length' meters ahead) before the ego can clear it,
        or standing in that stretch, blocks it. Once the ego is already mostly in the opposite
        lane it keeps going (stopping there would be a head-on standoff).
        """
        plan = self._plan_frame(ego_location, lookahead)
        if not plan:
            return False
        lane_width = plan[0][1].lane_width
        committed = abs(self._offset) > 0.5 * lane_width  # already committed to the bypass: keeps going

        clear_time = bypass_length / (self._bypass_speed / 3.6) + 2.0
        blocked = False
        for vehicle in vehicle_list:
            if vehicle.id == self._vehicle.id:
                continue
            location = vehicle.get_location()
            if location.distance(ego_location) > lookahead + 10.0:
                continue
            projection = self._project_on_plan(plan, location)
            if projection is None:
                continue
            s, d, wp = projection
            if s < -3.0 or not -(1.5 * lane_width + 0.5) < d < -0.5 * lane_width:
                continue  # behind the ego, or not in the opposite lane
            v_fwd = vehicle.get_transform().get_forward_vector()
            w_fwd = wp.transform.get_forward_vector()
            if v_fwd.x * w_fwd.x + v_fwd.y * w_fwd.y > -0.3:
                continue  # not coming towards the ego
            speed = get_speed(vehicle) / 3.6
            # Already in the bypass stretch, or reaching it before the ego can clear it
            vehicle_blocks = s < bypass_length or (speed > 0.5 and (s - bypass_length) / speed < clear_time)
            # Oncoming traffic shown in the dashboard: the vehicles in the bypass stretch or coming to it
            if s < bypass_length + 60.0 or vehicle_blocks:
                self._watch_oncoming(vehicle, vehicle_blocks and not committed)
            blocked = blocked or vehicle_blocks
        return blocked and not committed

    def _check_oncoming_for_left_turn(self, ego_wpt, ego_location, vehicle_list, time_threshold=4.0):
        """
        Check for oncoming vehicles that could interfere with a left turn.

        :param ego_wpt: current waypoint of ego
        :param ego_location: ego vehicle location
        :param vehicle_list: list of vehicles in the scene
        :param time_threshold: seconds threshold to consider vehicle relevant
        :return: (bool, target_vehicle, distance)
        """
        for target_vehicle in vehicle_list:
            
            if target_vehicle.id == self._vehicle.id:
                continue

            target_location = target_vehicle.get_location()
            target_wpt = self._map.get_waypoint(target_location, lane_type=carla.LaneType.Driving)

            # Must be on the same road and in opposite direction
            same_road = target_wpt.road_id == ego_wpt.road_id
            opposite_lane = target_wpt.lane_id * ego_wpt.lane_id < 0

            if not (same_road and opposite_lane):
                continue

            # Check if the vehicle is heading toward the junction
            target_transform = target_vehicle.get_transform()
            target_forward = target_transform.get_forward_vector()

            vector_to_junction = ego_wpt.transform.location - target_location
            vector_to_junction = carla.Vector3D(vector_to_junction.x, vector_to_junction.y, 0.0)
            dot_product = target_forward.x * vector_to_junction.x + target_forward.y * vector_to_junction.y

            if dot_product <= 0:
                continue  # Vehicle moving away from junction

            # Estimate time to reach the junction center
            distance = target_location.distance(ego_wpt.transform.location)
            speed = get_speed(target_vehicle)

            if speed < 0.1:
                continue  # Vehicle is stopped

            time_to_reach = distance / (speed + 1e-5)

            if time_to_reach < time_threshold:
                self._display.log(
                    f"Oncoming vehicle {target_vehicle.id} at {distance:.1f} m "
                    f"— ETA {time_to_reach:.1f} s", 'warning')
                return True, target_vehicle, distance

        return False, None, -1

    def _predict_conflict(self, ego_wpt, ego_location, vehicle_list, time_threshold=0.5):
        """
        Predict if any vehicle will enter the conflict zone of the junction during a left turn.
        """
        center_zone = self._get_center_commit_zone(ego_wpt, scale=0.5)
        ego_point = Point(ego_location.x, ego_location.y)
        self._draw_polygon(center_zone, z=0.5, color=(0, 255, 0), life_time=0.5)
        # Commit once ego enters the center zone
        if center_zone.contains(ego_point):
            if not self._left_turn_committed:
                self._display.log("Left turn committed — ego has entered intersection centre", 'action')
            self._left_turn_committed = True

        # If already committed, skip oncoming checks
        if self._left_turn_committed:
            self._left_turn_block_start = None
            return False, None, -1

        # If oncoming traffic has been blocking the turn continuously for too long,
        # force a commit instead of waiting forever (avoids a permanent livelock
        # when the opposite-direction flow never leaves a large enough gap).
        now = self._world.get_snapshot().timestamp.elapsed_seconds
        if self._left_turn_block_start is not None and now - self._left_turn_block_start > self._left_turn_max_wait:
            self._display.log(
                f"Left turn committed after {self._left_turn_max_wait:.0f}s wait — proceeding despite traffic",
                'action')
            self._left_turn_committed = True
            self._left_turn_block_start = None
            return False, None, -1

        # Continue with normal oncoming vehicle prediction
        conflict_polygon = self._get_conflict_polygon_speed(ego_wpt, 0)
        self._draw_polygon(conflict_polygon, z=1.0, color=(255, 0, 0), life_time=0.5)

        conflict = None
        for target_vehicle in vehicle_list:
            if target_vehicle.id == self._vehicle.id:
                continue

            target_transform = target_vehicle.get_transform()
            target_location = target_transform.location
            target_forward = target_transform.get_forward_vector()

            speed = get_speed(target_vehicle)
            if speed < 0.1:
                continue

            def reaches_conflict_zone(horizon):
                # Predict trajectory line
                current_point = Point(target_location.x, target_location.y)
                future_point = Point(target_location.x + target_forward.x * speed * horizon,
                                     target_location.y + target_forward.y * speed * horizon)
                return conflict_polygon.intersects(LineString([current_point, future_point]))

            in_conflict = reaches_conflict_zone(time_threshold)
            # Oncoming traffic shown in the dashboard: the vehicles heading to the conflict zone
            if in_conflict or reaches_conflict_zone(self._oncoming_watch_horizon):
                self._watch_oncoming(target_vehicle, in_conflict)

            if in_conflict and conflict is None:
                conflict = (target_vehicle, target_location.distance(ego_location))

        if conflict is not None:
            target_vehicle, distance = conflict
            if self._left_turn_block_start is None:
                self._left_turn_block_start = now
            self._display.log(
                f"Conflict predicted — vehicle {target_vehicle.id} will enter conflict zone", 'hazard')
            return True, target_vehicle, distance

        self._left_turn_block_start = None
        return False, None, -1

    def _watch_oncoming(self, vehicle, is_danger):
        """Records an oncoming vehicle the ego is taking into account, to show it in the dashboard"""
        previous = self._oncoming_watch.get(vehicle.id)
        self._oncoming_watch[vehicle.id] = (vehicle, is_danger or (previous is not None and previous[1]))

    def _get_conflict_polygon_speed(self, ego_wpt, ego_speed):
        """
        Returns a Polygon representing the conflict zone of the junction,
        extended in the ego vehicle's forward direction based on its speed.

        :param ego_wpt: current waypoint of the ego vehicle
        :param ego_speed: current speed of the ego vehicle in m/s
        """

        # Get junction bounding box
        junction = ego_wpt.get_junction()
        bbox = junction.bounding_box
        extent = bbox.extent

        cx, cy = bbox.location.x, bbox.location.y
        ex, ey = extent.x, extent.y

        # Define the base rectangle (junction bounding box)
        base_corners = [
            (cx + ex, cy + ey),
            (cx - ex, cy + ey),
            (cx - ex, cy - ey),
            (cx + ex, cy - ey),
        ]

        # Convert to list of points for extension
        corners = [Point(p) for p in base_corners]

        # Compute extension based on ego speed
        # Rule of thumb: extend in forward direction by (ego_speed × factor) meters
        # Use factor=1.5 for cautious behavior (can be tuned)
        extension_length = ego_speed * 1.5  # meters

        if extension_length > 0.0:
            forward_vec = ego_wpt.transform.get_forward_vector()

            # Extend the two front corners (assuming vehicle is approaching the junction from behind)
            # Which corners to extend depends on heading; for simplicity, extend all in forward direction
            extended_corners = []
            for point in corners:
                new_x = point.x + forward_vec.x * extension_length
                new_y = point.y + forward_vec.y * extension_length
                extended_corners.append((new_x, new_y))
        else:
            # If ego is stopped, use base junction box
            extended_corners = [(p.x, p.y) for p in corners]

        # Return the polygon
        return Polygon(extended_corners)

    def _get_conflict_polygon_speed(self, ego_wpt, ego_speed):
        """
        Returns a conflict polygon centered on the junction,
        optionally extended toward the junction center based on ego speed.

        :param ego_wpt: current waypoint of the ego vehicle
        :param ego_speed: current speed of the ego vehicle in m/s
        """
        junction = ego_wpt.get_junction()
        bbox = junction.bounding_box
        extent = bbox.extent

        cx, cy = bbox.location.x, bbox.location.y
        ex, ey = extent.x, extent.y

        # Base rectangle (junction box)
        base_corners = [
            (cx + ex, cy + ey),
            (cx - ex, cy + ey),
            (cx - ex, cy - ey),
            (cx + ex, cy - ey),
        ]

        # Optional extension: Expand the polygon outward in all directions (buffer)
        # This is better than using ego's forward vector because junctions have multiple entry points
        from shapely.geometry import Polygon
        polygon = Polygon(base_corners)

        # Expand the polygon to compensate for speed (simple safety margin)
        expansion_distance = ego_speed * 1.5  # meters
        if expansion_distance > 0.0:
            polygon = polygon.buffer(expansion_distance)

        return polygon
        
    def _get_center_commit_zone(self, ego_wpt, scale=0.3):
        """
        Returns a scaled polygon representing the center of the intersection.
        The size is a percentage of the junction bounding box.

        :param ego_wpt: current waypoint of the ego vehicle
        :param scale: fraction of the junction box to define the commit zone (e.g., 0.3)
        :return: shapely Polygon
        """
        junction = ego_wpt.get_junction()
        bbox = junction.bounding_box

        cx, cy = bbox.location.x, bbox.location.y
        ex, ey = bbox.extent.x, bbox.extent.y

        # Scale the box
        ex_scaled = ex * scale
        ey_scaled = ey * scale

        corners = [
            (cx + ex_scaled, cy + ey_scaled),
            (cx - ex_scaled, cy + ey_scaled),
            (cx - ex_scaled, cy - ey_scaled),
            (cx + ex_scaled, cy - ey_scaled),
        ]

        return Polygon(corners)

    def _path_band(self, ego_location, length):
        """
        The area the ego is about to drive over: a band along the plan (with the current lateral
        offset), from the ego to 'length' meters ahead, as wide as the ego plus a small margin
        """
        points = [(ego_location.x, ego_location.y)]
        for wp, _ in self._local_planner.get_plan():
            location = wp.transform.location
            if ego_location.distance(location) > length:
                break
            r_vec = wp.transform.get_right_vector()
            points.append((location.x + self._offset * r_vec.x, location.y + self._offset * r_vec.y))
        if len(points) < 2:
            return None
        return LineString(points).buffer(self._vehicle.bounding_box.extent.y + 0.3, cap_style=2)

    def _vehicle_obstacle_detected(self, vehicle_list=None, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        Method to check if there is a vehicle blocking the path.
        Also handles left-turn specific obstacle detection.
        """

        
        def get_route_polygon():
            route_bb = []
            extent_y = self._vehicle.bounding_box.extent.y
            r_ext = extent_y + self._offset
            l_ext = -extent_y + self._offset
            r_vec = ego_transform.get_right_vector()
            p1 = ego_location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
            p2 = ego_location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
            route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

            for wp, _ in self._local_planner.get_plan():
                if ego_location.distance(wp.transform.location) > max_distance:
                    break

                r_vec = wp.transform.get_right_vector()
                p1 = wp.transform.location + carla.Location(r_ext * r_vec.x, r_ext * r_vec.y)
                p2 = wp.transform.location + carla.Location(l_ext * r_vec.x, l_ext * r_vec.y)
                route_bb.extend([[p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z]])

            if len(route_bb) < 3:
                return None

            return Polygon(route_bb)

        if self._ignore_vehicles:
            return (False, None, -1)

        if not vehicle_list:
            vehicle_list = self._world.get_actors().filter("*vehicle*")

        if not max_distance:
            max_distance = self._base_vehicle_threshold

        ego_transform = self._vehicle.get_transform()
        ego_location = ego_transform.location
        ego_wpt = self._map.get_waypoint(ego_location)

        ego_front_transform = ego_transform
        ego_front_transform.location += carla.Location(
            self._vehicle.bounding_box.extent.x * ego_transform.get_forward_vector())

        opposite_invasion = abs(self._offset) + self._vehicle.bounding_box.extent.y > ego_wpt.lane_width / 2
        use_bbs = self._use_bbs_detection or opposite_invasion or ego_wpt.is_junction

        route_polygon = get_route_polygon()

        # --------- NEW LEFT TURN HANDLING ---------
        # get_plan() returns the ENTIRE remaining route (up to 10000 waypoints), ordered by
        # route progress (nearest first). Stop scanning once a waypoint falls outside the
        # 15m radius instead of skipping-and-continuing through the whole route - otherwise
        # a LEFT-tagged waypoint belonging to a LATER, unrelated junction that happens to be
        # geometrically close to the ego's CURRENT position (e.g. two junctions ~20m apart)
        # gets matched even though the ego is currently just going straight/right through a
        # different junction.
        left_turn_in_plan = False
        if ego_wpt.is_junction:
            for wp, road_option in self._local_planner.get_plan():
                if ego_location.distance(wp.transform.location) >= 15:
                    break
                if road_option == RoadOption.LEFT:
                    left_turn_in_plan = True
                    break
        
        ego_wpt = self._map.get_waypoint(self._vehicle.get_location())
        if not ego_wpt.is_junction:
            self._left_turn_committed = False
            self._left_turn_block_start = None
        
        # ---- EARLY ONCOMING VEHICLE CHECK FOR LEFT TURN ----
        if left_turn_in_plan and ego_wpt.is_junction:
            self._display.log("Approaching left turn — scanning for oncoming vehicles", 'info')
            blocked, target_vehicle, distance = self._predict_conflict(ego_wpt, ego_location, vehicle_list)
            if blocked:
                return (True, target_vehicle, distance)

        # Check for vehicle blocking the planned route (default logic)
        path_band = None  # built on first need (see _path_band)
        for target_vehicle in vehicle_list:
            if target_vehicle.id == self._vehicle.id:
                continue

            target_transform = target_vehicle.get_transform()
            if target_transform.location.distance(ego_location) > max_distance:
                continue

            target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)

            # A vehicle (moving or stopped) whose footprint overlaps the band the ego is about to
            # drive along is in the way, whatever lane / road it's registered on. This is checked
            # everywhere: the lane-based check below misses vehicles past a road boundary, on an
            # ending / merging lane or straddling two lanes, and the ego would drive into them.
            if path_band is None:
                path_band = self._path_band(ego_location, max_distance)
            if path_band is not None:
                target_vertices = target_vehicle.bounding_box.get_world_vertices(target_vehicle.get_transform())
                target_footprint = MultiPoint([(v.x, v.y) for v in target_vertices]).convex_hull
                if path_band.intersects(target_footprint):
                    return (True, target_vehicle, compute_distance(target_vehicle.get_location(), ego_location))

            # Junctions and lane invasions: the footprint check above is the one used
            if (use_bbs or target_wpt.is_junction) and route_polygon:
                continue

            # Simplified check (non-junction)
            else:
                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id + lane_offset:
                    next_wpt = self._local_planner.get_incoming_waypoint_and_direction(steps=3)[0]
                    if not next_wpt or next_wpt.is_junction:
                        # Lookahead crosses into a junction: road_id/lane_id numbering there
                        # doesn't reliably mean "same lane ahead", so don't use it to flag
                        # crossing/turning traffic as a same-lane obstacle (that's handled by
                        # the left-turn conflict check above, or should be ignored otherwise).
                        continue
                    if target_wpt.road_id != next_wpt.road_id or target_wpt.lane_id != next_wpt.lane_id + lane_offset:
                        continue

                target_forward_vector = target_transform.get_forward_vector()
                target_extent = target_vehicle.bounding_box.extent.x
                target_rear_transform = target_transform
                target_rear_transform.location -= carla.Location(
                    x=target_extent * target_forward_vector.x,
                    y=target_extent * target_forward_vector.y,
                )

                if is_within_distance(target_rear_transform, ego_front_transform, max_distance, [low_angle_th, up_angle_th]):
                    print("DIAGNOSTIC: vehicle-ahead hazard from simplified check - target={} "
                          "target_wpt(road_id={}, lane_id={}, is_junction={}) "
                          "ego_wpt(road_id={}, lane_id={}, is_junction={}) "
                          "distance={:.1f}m ego_loc={}".format(
                              target_vehicle.id, target_wpt.road_id, target_wpt.lane_id, target_wpt.is_junction,
                              ego_wpt.road_id, ego_wpt.lane_id, ego_wpt.is_junction,
                              compute_distance(target_transform.location, ego_transform.location), ego_location))
                    return (True, target_vehicle, compute_distance(target_transform.location, ego_transform.location))

        # --------- NO OBSTACLE IN LEFT TURN PATH ---------
        if left_turn_in_plan:
            self._display.log("Left turn path clear — proceeding", 'clear')

        return (False, None, -1)

    def _generate_lane_change_path(self, waypoint, direction='left', distance_same_lane=10,
                                distance_other_lane=25, lane_change_distance=25,
                                check=True, lane_changes=1, step_distance=2):
        """
        This methods generates a path that results in a lane change.
        Use the different distances to fine-tune the maneuver.
        If the lane change is impossible, the returned path will be empty.
        """
        distance_same_lane = max(distance_same_lane, 0.1)
        distance_other_lane = max(distance_other_lane, 0.1)
        lane_change_distance = max(lane_change_distance, 0.1)

        plan = []
        plan.append((waypoint, RoadOption.LANEFOLLOW))  # start position

        option = RoadOption.LANEFOLLOW

        # Same lane
        distance = 0
        while distance < distance_same_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        if direction == 'left':
            option = RoadOption.CHANGELANELEFT
        elif direction == 'right':
            option = RoadOption.CHANGELANERIGHT
        else:
            # ERROR, input value for change must be 'left' or 'right'
            return []

        lane_changes_done = 0
        lane_change_distance = lane_change_distance / lane_changes

        # Lane change
        while lane_changes_done < lane_changes:

            # Move forward
            next_wps = plan[-1][0].next(lane_change_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]

            # Get the side lane
            if direction == 'left':
                if check and str(next_wp.lane_change) not in ['Left', 'Both']:
                    return []
                side_wp = next_wp.get_left_lane()
            else:
                if check and str(next_wp.lane_change) not in ['Right', 'Both']:
                    return []
                side_wp = next_wp.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                return []

            # Update the plan
            plan.append((side_wp, option))
            lane_changes_done += 1

        # Other lane
        distance = 0
        while distance < distance_other_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        return plan
