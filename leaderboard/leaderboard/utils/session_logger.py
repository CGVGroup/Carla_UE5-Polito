"""
Session logger: records everything happening during a route driven by the NPC agent (BasicAgent2), with
or without a driver taking over, to analyze the sessions afterwards (statistics, metrics).

One folder per route, <log dir>/<routes file>_<visualization>_<date>/ (log dir: $SESSION_LOG_DIR, or
leaderboard/logs/sessions):

- frames.csv: one row per simulation frame. 'frame' is the CARLA frame (world.get_snapshot().frame), the
  same as in the driver's gaze recording (<CARLA Saved dir>/GazeLogs), to merge them
    time: frame, sim_time, wall_time
    ego: x, y, z, yaw, speed_kmh, accel (from the speed change, m/s2), accel_long / accel_lat / accel_vert
         (the simulation's acceleration in the vehicle's frame, lateral positive: right), road_id, lane_id,
         is_junction, lateral_offset (from the lane center, m, positive: right)
    control: applied throttle / steer / brake / hand_brake / reverse, the agent's command, steer_deg /
         cmd_steer_deg (front wheels' angle), control_source ('agent' or 'driver': the driver took over, the
         applied control isn't the agent's) and takeover_cause (throttle / steer / brake). The server's own
         takeover state and cause, and the driver's raw pedals / wheel every frame, are in the gaze
         recording (automation_state, takeover_cause, driver_throttle / driver_brake / driver_steer)
    interface: detections (id:category:danger of what the system takes into account, whatever the mode:
         the content of the 'full' mode's boxes), rendered_boxes (id:box:danger of the boxes drawn)
    scenario road users (role 'scenario'): hazards_visible_onset (ids first in the driver's line of sight
         this frame: t0), scenario_min_ttc / _id / _gap (lowest time to collision with one this frame)
    agent: failure_behavior, hazards (what it stops / brakes for), emergency_stop, target_speed and the
         speed limits it follows, construction_phase, planner_offset, road_option, detected obstacle ids
    surroundings: nearest_vehicle_dist, nearest_pedestrian_dist, lead_vehicle_dist, lead_vehicle_ttc,
         traffic_light_state
    context: scenario (the route scenario the ego is in), dashboard mode and text
- events.jsonl: one JSON object per event: route / scenario start and end, takeover start (with its
  cause) / end (re-engagement), collision, lane_invasion, hard_brake, failure_behavior on / off,
  llm_trigger, emergency_stop start / end, hazard_visible (visibility onset t0 of a scenario road user,
  with its distance, TTC, and whether the system detected it), gaze_check
- summary.json: metrics of the whole route and of each scenario (duration, distance, speeds, max
  deceleration, hard brakes, collisions, lane invasions, takeovers and their duration, emergency stops,
  minimum distances / time to collision, mean absolute lateral offset...)
- recording.log: the CARLA recorder's recording of the whole simulation
  (https://carla.readthedocs.io/en/latest/adv_recorder/), with the additional data: every actor's
  creation / destruction and transform every frame, plus bounding boxes, vehicle controls, physics
  controls, lights, traffic light states, walker animations, platform time. It can be replayed
  (client.replay_file) and queried afterwards. At the end of the route, the recorder's own analyses are
  saved next to it: recording_info.txt (actors and events), recording_collisions.txt (every collision
  between any actors), recording_blocked.txt (actors that stayed blocked, e.g. stuck vehicles).
  Not started when the leaderboard records itself (RECORD_PATH / --record): only one recording at a time
"""

import csv
import datetime
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.utils import step_timing


def _now_iso():
    return datetime.datetime.now().isoformat(timespec='milliseconds')


def _speed(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


class _Scenarios:
    """
    The route's scenarios (from the routes XML), to know which one the ego is in: a scenario starts when
    the ego gets within START_RADIUS of its trigger point, ends when it gets farther than its extent
    (its 'distance' parameter, if any, plus END_MARGIN) or when the next one starts
    """
    START_RADIUS = 15.0
    END_MARGIN = 60.0

    def __init__(self, routes_file, route_id=None):
        self.items = []  # [(name, type, carla.Location, extent)]
        self.current = None  # index in items
        try:
            root = ET.parse(routes_file).getroot()
        except (OSError, ET.ParseError):
            return
        for route in root.iter('route'):
            if route_id is not None and route.attrib.get('id') != str(route_id):
                continue
            scenarios = route.find('scenarios')
            if scenarios is None:
                continue
            for scenario in scenarios.iter('scenario'):
                trigger = scenario.find('trigger_point')
                if trigger is None:
                    continue
                location = carla.Location(float(trigger.attrib['x']), float(trigger.attrib['y']),
                                          float(trigger.attrib['z']))
                distance = scenario.find('distance')
                extent = float(distance.attrib.get('value', 0)) if distance is not None else 0.0
                self.items.append((scenario.attrib.get('name', '?'), scenario.attrib.get('type', '?'),
                                   location, extent + self.END_MARGIN))
            break

    def update(self, location):
        """Returns (current scenario name or '', [(event, name, type)])"""
        events = []
        if self.current is not None:
            name, kind, trigger, extent = self.items[self.current]
            if location.distance(trigger) > extent:
                events.append(('scenario_end', name, kind))
                self.current = None
        for index, (name, kind, trigger, _) in enumerate(self.items):
            if index != self.current and location.distance(trigger) < self.START_RADIUS:
                if self.current is not None:
                    events.append(('scenario_end',) + self.items[self.current][:2])
                self.current = index
                events.append(('scenario_start', name, kind))
                break
        return (self.items[self.current][0] if self.current is not None else ''), events


class _Hazards:
    """
    The scenario's road users (vehicles, bikes, pedestrians spawned by the scenarios, role 'scenario'):
    - visibility onset (t0): the first frame the driver has an unobstructed line of sight to one, checked
      with ray casts from the driver's eyes to its center and its top. Only for the ones not seen yet,
      within RANGE ahead, nearest first, at most MAX_RAYS per frame (each ray is a call to the server)
    - time to collision with each, every frame (from the frame's snapshot: no call to the server):
      gap between the two bounding boxes (approximated by circles) / closing speed, when closing
    """
    RANGE = 100.0  # [m]
    FIELD_OF_VIEW = 100.0  # [deg] each side of the ego's forward direction
    MAX_RAYS = 6
    EGO_HULL = 3.0  # [m] hits closer to the eyes than this are the ego vehicle itself (windshield, hood)
    EYES = carla.Location(x=0.0, y=-0.35, z=1.2)  # driver's eyes in the ego vehicle's frame (left-hand drive)

    def __init__(self, world, ego):
        self._world = world
        self._ego = ego
        self._ego_radius = max(ego.bounding_box.extent.x, ego.bounding_box.extent.y)
        self.actors = {}  # id -> (actor, radius)
        self.visible = {}  # id -> sim time of the visibility onset
        self.per_actor = {}  # id -> stats of the actor (see summary)

    def refresh(self, actors):
        """Scenario road users among the actors (carla.ActorList)"""
        current = {}
        for actor in list(actors.filter('vehicle.*')) + list(actors.filter('walker.pedestrian.*')):
            if actor.id != self._ego.id and actor.attributes.get('role_name') == 'scenario':
                box = actor.bounding_box
                if actor.type_id.startswith('walker.'):
                    center, top = 0.0, 0.7 * box.extent.z  # located at the center of the body
                else:
                    center = box.location.z if box.location.z > 0.1 else box.extent.z  # located at its base
                    top = 1.7 * center
                current[actor.id] = (actor, max(box.extent.x, box.extent.y), center, top)
        self.actors = current

    def _clear_line(self, eyes, target, target_center, target_radius):
        """Whether nothing (but the ego and the target) is between the eyes and the target point"""
        try:
            hits = self._world.cast_ray(eyes, target)
        except RuntimeError:
            return False
        distance = eyes.distance(target)
        for hit in hits:
            d = eyes.distance(hit.location)
            if d < self.EGO_HULL or d > distance - 0.1:
                continue
            if hit.location.distance(target_center) <= target_radius + 0.3:
                continue  # the target itself
            return False
        return True

    def tick(self, snapshot, ego_transform, ego_velocity, sim_time):
        """
        Returns ([(id, type, distance) of the ones becoming visible this frame], (min ttc, its actor id,
        min gap) over the scenario road users within RANGE)
        """
        ego_location = ego_transform.location
        forward = ego_transform.get_forward_vector()
        eyes = ego_transform.transform(self.EYES)
        candidates = []
        min_ttc = (None, None, None)
        for actor_id, (actor, radius, center_height, top_height) in self.actors.items():
            actor_snapshot = snapshot.find(actor_id)
            if actor_snapshot is None:
                continue
            location = actor_snapshot.get_transform().location
            dx, dy = location.x - ego_location.x, location.y - ego_location.y
            distance = math.hypot(dx, dy)
            if distance > self.RANGE:
                continue
            # Time to collision
            velocity = actor_snapshot.get_velocity()
            rvx, rvy = velocity.x - ego_velocity.x, velocity.y - ego_velocity.y
            closing = -(dx * rvx + dy * rvy) / max(distance, 1e-3)
            gap = max(0.0, distance - self._ego_radius - radius)
            ttc = gap / closing if closing > 0.1 else None
            stats = self.per_actor.setdefault(actor_id, {
                'type': actor.type_id, 'first_in_range': round(sim_time, 3), 'visible_at': None,
                'distance_at_visible': None, 'ttc_at_visible': None, 'min_gap': None, 'min_ttc': None,
                'min_ttc_time': None})
            if stats['min_gap'] is None or gap < stats['min_gap']:
                stats['min_gap'] = round(gap, 3)
            if ttc is not None and (stats['min_ttc'] is None or ttc < stats['min_ttc']):
                stats['min_ttc'], stats['min_ttc_time'] = round(ttc, 3), round(sim_time, 3)
            if ttc is not None and (min_ttc[0] is None or ttc < min_ttc[0]):
                min_ttc = (ttc, actor_id, gap)
            # Not seen yet, and ahead: line of sight to check
            if actor_id not in self.visible and distance > 0.1 \
                    and (dx * forward.x + dy * forward.y) / distance > math.cos(math.radians(self.FIELD_OF_VIEW)):
                candidates.append((distance, actor_id, actor, radius, location, ttc, center_height, top_height))

        new = []
        rays = 0
        for distance, actor_id, actor, radius, location, ttc, center_height, top_height in \
                sorted(candidates, key=lambda c: c[0]):
            if rays + 2 > self.MAX_RAYS:
                break
            center = location + carla.Location(z=center_height)
            top = location + carla.Location(z=top_height)
            rays += 1
            seen = self._clear_line(eyes, center, center, radius)
            if not seen:
                rays += 1
                seen = self._clear_line(eyes, top, center, radius)
            if seen:
                self.visible[actor_id] = sim_time
                stats = self.per_actor[actor_id]
                stats['visible_at'] = round(sim_time, 3)
                stats['distance_at_visible'] = round(distance, 2)
                stats['ttc_at_visible'] = _round(ttc)
                new.append((actor_id, actor.type_id, distance, ttc))
        return new, min_ttc


class SessionLogger:
    """Records a route driven by the agent (see the module's documentation)"""

    HARD_BRAKE_DECEL = 4.0  # [m/s2] deceleration from which a braking is a 'hard brake'
    CONTROL_TOLERANCE = 0.02  # applied control differing more from the agent's command: not the agent's
    DRIVER_FRAMES = 3  # frames of differing control before counting it as the driver's
    AGENT_FRAMES = 10  # frames of the agent's control again before counting it back as the agent's
    NEARBY_RADIUS = 60.0  # [m] other road users considered for the minimum distances
    ACTORS_REFRESH = 10  # frames between two fetches of the vehicles / pedestrians lists

    FIELDS = [
        'frame', 'sim_time', 'wall_time',
        'x', 'y', 'z', 'yaw', 'speed_kmh', 'accel', 'accel_long', 'accel_lat', 'accel_vert',
        'road_id', 'lane_id', 'is_junction', 'lateral_offset',
        'throttle', 'steer', 'steer_deg', 'brake', 'hand_brake', 'reverse',
        'cmd_throttle', 'cmd_steer', 'cmd_steer_deg', 'cmd_brake', 'control_source', 'takeover_cause',
        'failure_behavior', 'hazards', 'emergency_stop', 'target_speed', 'curve_speed_limit',
        'light_speed_limit', 'stop_sign_speed_limit', 'obstacle_speed_limit', 'construction_phase',
        'construction_room', 'planner_offset', 'road_option', 'vehicle_obstacle_id', 'pedestrian_obstacle_id',
        'traffic_light_id', 'oncoming_watched',
        'nearest_vehicle_dist', 'nearest_pedestrian_dist', 'lead_vehicle_dist', 'lead_vehicle_ttc',
        'traffic_light_state',
        'scenario', 'dashboard_mode', 'dashboard_text', 'detections', 'rendered_boxes',
        'hazards_visible_onset', 'scenario_min_ttc', 'scenario_min_ttc_id', 'scenario_min_gap',
    ]

    def __init__(self, vehicle, name, visualization, routes_file=None, log_dir=None):
        self._vehicle = vehicle
        self._world = vehicle.get_world()
        self._map = CarlaDataProvider.get_map()
        log_dir = log_dir or os.environ.get('SESSION_LOG_DIR') or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'logs', 'sessions')
        self.directory = os.path.join(log_dir, '{}_{}'.format(name, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
        os.makedirs(self.directory, exist_ok=True)

        self._frames_file = open(os.path.join(self.directory, 'frames.csv'), 'w', newline='', encoding='utf-8')
        self._frames = csv.DictWriter(self._frames_file, fieldnames=self.FIELDS)
        self._frames.writeheader()
        self._events_file = open(os.path.join(self.directory, 'events.jsonl'), 'w', encoding='utf-8')
        self._lock = threading.Lock()  # events also come from the sensors' threads
        self._last_flush = time.monotonic()

        self._scenarios = _Scenarios(routes_file) if routes_file else None
        self._info = {'name': name, 'visualization': visualization, 'routes_file': routes_file,
                      'ego_id': vehicle.id, 'ego_type': vehicle.type_id, 'start': _now_iso(),
                      'gaze_recording': '<CARLA Saved dir>/GazeLogs/{}_<date>.csv (packaged CARLA: '
                                        '%LOCALAPPDATA%/CarlaUnreal/Saved)'.format(name)}

        # State
        self._frame = None
        # Wall clock / simulation time of the first and last frames logged: the real-time factor
        self._first_wall = self._first_sim = self._last_wall = None
        step_timing.reset()  # this route's steps only
        self._nearby_actors = None  # (frame fetched, vehicle ids, pedestrian ids)
        self._sim_time = None
        self._previous_speed = None
        self._previous_command = None
        self._driver = False
        self._mismatch_frames = 0
        self._match_frames = 0
        self._takeover_start = None
        self._failure = False
        self._emergency = False
        self._hard_braking = False
        self._scenario = ''
        self._takeover_cause = ''  # of the current takeover
        self._pending_cause = ''  # input deviating most when the applied control starts deviating
        self._hazards = _Hazards(self._world, vehicle)

        # Steering in degrees: steer (-1..1) x the front wheels' maximum steering angle (read once)
        self._max_steer_deg = None
        try:
            wheels = vehicle.get_physics_control().wheels
            self._max_steer_deg = max(w.max_steer_angle for w in wheels[:2]) if wheels else None
        except (RuntimeError, AttributeError, ValueError):
            pass
        self._info['max_steer_angle_deg'] = self._max_steer_deg

        # Statistics (whole route and per scenario)
        self._stats = {'': self._new_stats()}
        self._takeovers = []
        self._collisions = []

        # Sensors: collisions and lane invasions of the ego
        self._sensors = []
        library = self._world.get_blueprint_library()
        for blueprint_id, callback in (('sensor.other.collision', self._on_collision),
                                       ('sensor.other.lane_invasion', self._on_lane_invasion)):
            try:
                sensor = self._world.spawn_actor(library.find(blueprint_id), carla.Transform(), attach_to=vehicle)
                sensor.listen(callback)
                self._sensors.append(sensor)
            except (RuntimeError, IndexError) as e:
                print('Session logger: no {} ({})'.format(blueprint_id, e))

        # CARLA recorder: the whole simulation, with all the additional data
        self._client = CarlaDataProvider.get_client()
        self._recording = None
        if os.environ.get('RECORD_PATH'):
            print('Session logger: the leaderboard records the simulation itself (RECORD_PATH), not recorded here')
        elif self._client is not None:
            path = os.path.abspath(os.path.join(self.directory, 'recording.log'))
            try:
                answer = self._client.start_recorder(path, True)
                self._recording = path
                self._info['recording'] = path
                print('Session logger: CARLA recorder started ({})'.format(answer))
            except RuntimeError as e:
                print('Session logger: CARLA recorder not started ({})'.format(e))

        self.event('route_start', **self._info)
        print('Session logger: recording to {}'.format(self.directory))

    # ------------------------------------------------------------------ statistics

    @staticmethod
    def _new_stats():
        return {'frames': 0, 'duration': 0.0, 'distance': 0.0, 'speed_sum': 0.0, 'max_speed_kmh': 0.0,
                'min_speed_kmh': None, 'max_decel': 0.0, 'hard_brakes': 0, 'collisions': 0,
                'lane_invasions': 0, 'takeovers': 0, 'takeover_time': 0.0, 'emergency_stops': 0,
                'stopped_time': 0.0, 'min_vehicle_dist': None, 'min_pedestrian_dist': None,
                'min_lead_ttc': None, 'min_scenario_ttc': None, 'abs_lateral_offset_sum': 0.0, 'driver_frames': 0,
                'failure_frames': 0}

    def _stats_for(self, scenario):
        targets = [self._stats['']]
        if scenario:
            targets.append(self._stats.setdefault(scenario, self._new_stats()))
        return targets

    @staticmethod
    def _min(current, value):
        if value is None:
            return current
        return value if current is None else min(current, value)

    def _steer_deg(self, steer):
        """Front wheels' steering angle [deg] of a steer value (-1..1), None if unknown"""
        if steer is None or self._max_steer_deg is None:
            return None
        return round(steer * self._max_steer_deg, 2)

    def _nearest(self, snapshot, actor_ids, location):
        """Distance to the nearest of the actors within NEARBY_RADIUS in the snapshot (None if none)"""
        nearest = None
        for actor_id in actor_ids:
            actor_snapshot = snapshot.find(actor_id)
            if actor_snapshot is not None:
                d = actor_snapshot.get_transform().location.distance(location)
                if d < self.NEARBY_RADIUS:
                    nearest = self._min(nearest, d)
        return nearest

    # ------------------------------------------------------------------ events

    def event(self, kind, **data):
        record = {'event': kind, 'frame': self._frame, 'sim_time': _round(self._sim_time), 'wall_time': _now_iso()}
        try:
            location = self._vehicle.get_location()
            record.update({'x': round(location.x, 2), 'y': round(location.y, 2)})
        except RuntimeError:
            pass
        record['scenario'] = self._scenario
        record.update(data)
        with self._lock:
            if self._events_file is not None:
                self._events_file.write(json.dumps(record, default=str) + '\n')

    def _on_collision(self, event):
        other = event.other_actor
        impulse = event.normal_impulse
        data = {'other_id': other.id if other else None, 'other_type': other.type_id if other else None,
                'impulse': round(math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2), 1)}
        self._collisions.append(dict(data, frame=event.frame, scenario=self._scenario))
        self.event('collision', **data)
        for stats in self._stats_for(self._scenario):
            stats['collisions'] += 1

    def _on_lane_invasion(self, event):
        markings = sorted({str(m.type).split('.')[-1] for m in event.crossed_lane_markings})
        self.event('lane_invasion', markings=markings)
        for stats in self._stats_for(self._scenario):
            stats['lane_invasions'] += 1

    # ------------------------------------------------------------------ per frame

    def tick(self, step_info, failure_behavior=False, dashboard=None):
        """Records the current frame (called at each agent step)"""
        try:
            snapshot = self._world.get_snapshot()
            transform = self._vehicle.get_transform()
            control = self._vehicle.get_control()
            speed = _speed(self._vehicle)
        except RuntimeError:
            return
        if snapshot.frame == self._frame:
            return
        dt = snapshot.timestamp.delta_seconds or 0.05
        self._frame, self._sim_time = snapshot.frame, snapshot.timestamp.elapsed_seconds
        self._last_wall = time.time()
        if self._first_wall is None:
            self._first_wall, self._first_sim = self._last_wall, self._sim_time
        location = transform.location
        info = step_info or {}

        # Who drives: the control applied by the server this frame is the agent's previous command,
        # unless the driver took over (then it's the driver's wheel / pedals)
        if self._previous_command is not None:
            throttle, steer, brake = self._previous_command
            deviations = {'throttle': abs(control.throttle - throttle), 'steer': abs(control.steer - steer),
                          'brake': abs(control.brake - brake)}
            differs = max(deviations.values()) > self.CONTROL_TOLERANCE
            self._mismatch_frames = self._mismatch_frames + 1 if differs else 0
            self._match_frames = 0 if differs else self._match_frames + 1
            if differs and self._mismatch_frames == 1 and not self._driver:
                # Cause: the input deviating most from the agent's when it starts deviating
                self._pending_cause = max(deviations, key=deviations.get)
            if not self._driver and self._mismatch_frames >= self.DRIVER_FRAMES:
                self._driver = True
                self._takeover_start = self._sim_time
                self._takeover_cause = self._pending_cause
                # (the server's exact state and cause, and the driver's raw pedals / wheel every frame,
                # are in the gaze recording: automation_state, takeover_cause, driver_* columns)
                self.event('takeover_start', speed_kmh=round(speed * 3.6, 1), cause=self._takeover_cause)
                for stats in self._stats_for(self._scenario):
                    stats['takeovers'] += 1
            elif self._driver and self._match_frames >= self.AGENT_FRAMES:
                self._driver = False
                duration = self._sim_time - (self._takeover_start or self._sim_time)
                self._takeovers.append({'start': _round(self._takeover_start), 'duration': round(duration, 2),
                                        'scenario': self._scenario, 'cause': self._takeover_cause})
                self.event('takeover_end', duration=round(duration, 2))  # re-engagement
                self._takeover_cause = ''
        self._previous_command = info.get('command')

        # Scenario
        if self._scenarios is not None:
            self._scenario, events = self._scenarios.update(location)
            for kind, name, scenario_type in events:
                self.event(kind, name=name, type=scenario_type)

        # Accelerations: 'accel' from the speed change (hard brakes), and the vehicle's own acceleration
        # (from the simulation, not differentiated) in its frame: longitudinal, lateral (positive: right),
        # vertical
        accel = 0.0 if self._previous_speed is None else (speed - self._previous_speed) / dt
        self._previous_speed = speed
        try:
            a = self._vehicle.get_acceleration()
        except RuntimeError:
            a = carla.Vector3D()
        forward, right, up = transform.get_forward_vector(), transform.get_right_vector(), transform.get_up_vector()
        accel_long = a.x * forward.x + a.y * forward.y + a.z * forward.z
        accel_lat = a.x * right.x + a.y * right.y + a.z * right.z
        accel_vert = a.x * up.x + a.y * up.y + a.z * up.z
        if accel < -self.HARD_BRAKE_DECEL and not self._hard_braking:
            self._hard_braking = True
            self.event('hard_brake', decel=round(-accel, 2), speed_kmh=round(speed * 3.6, 1))
            for stats in self._stats_for(self._scenario):
                stats['hard_brakes'] += 1
        elif accel > -self.HARD_BRAKE_DECEL / 2:
            self._hard_braking = False

        # Agent's state changes
        if info.get('failure_behavior', failure_behavior) != self._failure:
            self._failure = not self._failure
            self.event('failure_behavior_on' if self._failure else 'failure_behavior_off')
        emergency = bool(info.get('emergency_stop'))
        if emergency != self._emergency:
            self._emergency = emergency
            self.event('emergency_stop_start' if emergency else 'emergency_stop_end', hazards=info.get('hazards'))
            if emergency:
                for stats in self._stats_for(self._scenario):
                    stats['emergency_stops'] += 1

        # Surroundings
        # (the ids of the vehicles / pedestrians are fetched every ACTORS_REFRESH frames only, their
        # locations are read from this frame's snapshot)
        if self._nearby_actors is None or self._frame - self._nearby_actors[0] >= self.ACTORS_REFRESH:
            actors = self._world.get_actors()
            self._nearby_actors = (self._frame,
                                   [a.id for a in actors.filter('vehicle.*') if a.id != self._vehicle.id],
                                   [a.id for a in actors.filter('walker.pedestrian.*')])
            self._hazards.refresh(actors)
        nearest_vehicle = self._nearest(snapshot, self._nearby_actors[1], location)
        nearest_pedestrian = self._nearest(snapshot, self._nearby_actors[2], location)

        # Scenario road users: visibility onset (t0) and time to collision
        visible_onsets, (scenario_ttc, scenario_ttc_id, scenario_gap) = self._hazards.tick(
            snapshot, transform, self._vehicle.get_velocity(), self._sim_time)
        for actor_id, type_id, distance, ttc in visible_onsets:
            self.event('hazard_visible', actor_id=actor_id, type=type_id, distance=round(distance, 2),
                       ttc=_round(ttc), detected=any(d[0] == actor_id for d in info.get('detections') or []))
        for stats in self._stats_for(self._scenario):
            stats['min_scenario_ttc'] = self._min(stats['min_scenario_ttc'], scenario_ttc)
        lead_dist, lead_ttc = None, None
        lead_id = info.get('vehicle_obstacle_id')
        if lead_id is not None:
            lead = self._world.get_actor(lead_id)
            if lead is not None:
                lead_dist = lead.get_location().distance(location)
                forward = transform.get_forward_vector()
                v_ego, v_lead = self._vehicle.get_velocity(), lead.get_velocity()
                closing = (v_ego.x - v_lead.x) * forward.x + (v_ego.y - v_lead.y) * forward.y
                if closing > 0.1:
                    lead_ttc = lead_dist / closing
        try:
            light_state = str(self._vehicle.get_traffic_light_state()).split('.')[-1]
        except RuntimeError:
            light_state = ''

        waypoint = self._map.get_waypoint(location)
        row = {
            'frame': self._frame, 'sim_time': round(self._sim_time, 3), 'wall_time': _now_iso(),
            'x': round(location.x, 3), 'y': round(location.y, 3), 'z': round(location.z, 3),
            'yaw': round(transform.rotation.yaw, 2), 'speed_kmh': round(speed * 3.6, 2), 'accel': round(accel, 3),
            'accel_long': round(accel_long, 3), 'accel_lat': round(accel_lat, 3), 'accel_vert': round(accel_vert, 3),
            'road_id': waypoint.road_id, 'lane_id': waypoint.lane_id, 'is_junction': int(waypoint.is_junction),
            'lateral_offset': _round(info.get('lateral_offset')),
            'throttle': round(control.throttle, 3), 'steer': round(control.steer, 3),
            'steer_deg': self._steer_deg(control.steer), 'brake': round(control.brake, 3),
            'hand_brake': int(control.hand_brake), 'reverse': int(control.reverse),
            'cmd_throttle': _round((info.get('command') or (None,) * 3)[0]),
            'cmd_steer': _round((info.get('command') or (None,) * 3)[1]),
            'cmd_steer_deg': self._steer_deg((info.get('command') or (None,) * 3)[1]),
            'cmd_brake': _round((info.get('command') or (None,) * 3)[2]),
            'control_source': 'driver' if self._driver else 'agent',
            'takeover_cause': self._takeover_cause if self._driver else '',
            'failure_behavior': int(self._failure), 'hazards': '|'.join(info.get('hazards') or []),
            'emergency_stop': int(emergency), 'target_speed': _round(info.get('target_speed')),
            'curve_speed_limit': _round(info.get('curve_speed_limit')),
            'light_speed_limit': _round(info.get('light_speed_limit')),
            'stop_sign_speed_limit': _round(info.get('stop_sign_speed_limit')),
            'obstacle_speed_limit': _round(info.get('obstacle_speed_limit')),
            'construction_phase': info.get('construction_phase') or '',
            'construction_room': _round(info.get('construction_room')),
            'planner_offset': _round(info.get('planner_offset')), 'road_option': info.get('road_option', ''),
            'vehicle_obstacle_id': info.get('vehicle_obstacle_id'), 'pedestrian_obstacle_id': info.get('pedestrian_obstacle_id'),
            'traffic_light_id': info.get('traffic_light_id'), 'oncoming_watched': info.get('oncoming_watched'),
            'nearest_vehicle_dist': _round(nearest_vehicle), 'nearest_pedestrian_dist': _round(nearest_pedestrian),
            'lead_vehicle_dist': _round(lead_dist), 'lead_vehicle_ttc': _round(lead_ttc),
            'traffic_light_state': light_state, 'scenario': self._scenario,
            'dashboard_mode': getattr(dashboard, 'mode', '') if dashboard else '',
            'dashboard_text': (getattr(dashboard, '_text', '') or '') if dashboard else '',
            # id:category:danger of what the system takes into account / id:box:danger of the boxes drawn
            'detections': '|'.join('{}:{}:{}'.format(i, c, int(d)) for i, c, d in info.get('detections') or []),
            'rendered_boxes': '|'.join('{}:{}:{}'.format(i, t, int(d)) for i, t, d in
                                       (dashboard.rendered_boxes() if hasattr(dashboard, 'rendered_boxes') else [])),
            'hazards_visible_onset': '|'.join(str(v[0]) for v in visible_onsets),
            'scenario_min_ttc': _round(scenario_ttc), 'scenario_min_ttc_id': scenario_ttc_id,
            'scenario_min_gap': _round(scenario_gap),
        }
        self._frames.writerow(row)

        for stats in self._stats_for(self._scenario):
            stats['frames'] += 1
            stats['duration'] += dt
            stats['distance'] += speed * dt
            stats['speed_sum'] += speed * 3.6
            stats['max_speed_kmh'] = max(stats['max_speed_kmh'], speed * 3.6)
            stats['min_speed_kmh'] = self._min(stats['min_speed_kmh'], speed * 3.6)
            stats['max_decel'] = max(stats['max_decel'], -accel)
            if speed < 0.3:
                stats['stopped_time'] += dt
            if self._driver:
                stats['takeover_time'] += dt
                stats['driver_frames'] += 1
            if self._failure:
                stats['failure_frames'] += 1
            stats['min_vehicle_dist'] = self._min(stats['min_vehicle_dist'], nearest_vehicle)
            stats['min_pedestrian_dist'] = self._min(stats['min_pedestrian_dist'], nearest_pedestrian)
            stats['min_lead_ttc'] = self._min(stats['min_lead_ttc'], lead_ttc)
            if info.get('lateral_offset') is not None:
                stats['abs_lateral_offset_sum'] += abs(info['lateral_offset'])

        if time.monotonic() - self._last_flush > 1.0:
            self._frames_file.flush()
            with self._lock:
                self._events_file.flush()
            self._last_flush = time.monotonic()

    # ------------------------------------------------------------------ end

    def _stop_recording(self):
        """Stops the CARLA recorder and saves its analyses of the recording next to it"""
        if self._recording is None or self._client is None:
            return
        try:
            self._client.stop_recorder()
        except RuntimeError as e:
            print('Session logger: CARLA recorder not stopped ({})'.format(e))
            return
        # The analyses are made by the server reading the recording back (within the client's timeout,
        # 300 s in the leaderboard)
        base = os.path.splitext(self._recording)[0]
        analyses = (
            ('info', lambda: self._client.show_recorder_file_info(self._recording, False)),
            ('collisions', lambda: self._client.show_recorder_collisions(self._recording, 'a', 'a')),
            ('blocked', lambda: self._client.show_recorder_actors_blocked(self._recording, 30.0, 10.0)),
        )
        for name, query in analyses:
            try:
                with open('{}_{}.txt'.format(base, name), 'w', encoding='utf-8') as f:
                    f.write(query())
            except (RuntimeError, OSError) as e:
                print('Session logger: no recorder {} analysis ({})'.format(name, e))
        print('Session logger: CARLA recording saved to {}'.format(self._recording))
        self._recording = None

    def close(self):
        """End of the route: writes the summary, closes the files, removes the sensors"""
        if self._frames_file is None:
            return
        for sensor in self._sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except RuntimeError:
                pass
        self._sensors = []
        if self._driver:
            duration = (self._sim_time or 0.0) - (self._takeover_start or self._sim_time or 0.0)
            self._takeovers.append({'start': _round(self._takeover_start), 'duration': round(duration, 2),
                                    'scenario': self._scenario, 'until_end': True})
            self.event('takeover_end', duration=round(duration, 2), until_end=True)
        if self._scenario:
            self.event('scenario_end', name=self._scenario)
        self.event('route_end')
        self._stop_recording()

        def finish(stats):
            frames = max(1, stats['frames'])
            out = dict(stats)
            out['mean_speed_kmh'] = round(stats['speed_sum'] / frames, 2)
            out['mean_abs_lateral_offset'] = round(stats['abs_lateral_offset_sum'] / frames, 3)
            out['takeover_ratio'] = round(stats['driver_frames'] / frames, 3)
            for key in ('speed_sum', 'abs_lateral_offset_sum'):
                del out[key]
            return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in out.items()}

        summary = dict(self._info)
        summary['end'] = _now_iso()
        summary['route'] = finish(self._stats[''])
        summary['scenarios'] = {name: finish(stats) for name, stats in self._stats.items() if name}
        # Log rate: the simulation step (--frame-rate). Real-time factor: simulation time / wall clock time
        # (1: real time; below: the server and the agent didn't keep up with the frame rate, slow motion)
        try:
            step = self._world.get_settings().fixed_delta_seconds
        except RuntimeError:
            step = None
        summary['frame_rate'] = round(1.0 / step, 2) if step else None
        wall = (self._last_wall - self._first_wall) if self._first_wall is not None else 0.0
        sim = (self._sim_time - self._first_sim) if self._first_wall is not None else 0.0
        summary['realtime_factor'] = round(sim / wall, 3) if wall > 0 else None
        summary['achieved_log_rate'] = round(max(0, self._stats['']['frames'] - 1) / wall, 2) if wall > 0 else None
        # Mean ms per step and part (see step_timing): what limits the steps per second
        summary['step_timing_ms'] = step_timing.summary()
        if summary['realtime_factor'] is not None and summary['realtime_factor'] < 0.95:
            print('Session logger: the simulation ran at {:.0%} of real time ({} frames/s for {} Hz)'.format(
                summary['realtime_factor'], summary['achieved_log_rate'], summary['frame_rate']))
        summary['takeovers'] = self._takeovers
        summary['collisions'] = self._collisions
        # Per scenario road user: when it came in range / became visible (t0), distance and TTC then,
        # minimum gap and TTC
        summary['scenario_actors'] = {str(i): s for i, s in self._hazards.per_actor.items()}
        with open(os.path.join(self.directory, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, default=str)

        self._frames_file.close()
        with self._lock:
            self._events_file.close()
            self._events_file = None
        self._frames_file = None
        print('Session logger: summary written to {}'.format(os.path.join(self.directory, 'summary.json')))


def _round(value, digits=3):
    return None if value is None else round(value, digits)
