#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function

import math
import py_trees
import carla
from math import floor

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      KeepVelocity,
                                                                      Idle,
                                                                      ActorTransformSetter,
                                                                      MovePedestrianWithEgo)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocation,
                                                                               InTimeToArrivalToLocation,
                                                                               DriveDistance)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp

from srunner.tools.background_manager import LeaveSpaceInFront, LeaveCrossingSpace


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default


class CrossingWalker(object):
    """
    Moves the crossing pedestrian along a straight line with walker states (location +
    speed), sampling the ground height along the way so it steps down/up the curbs.

    This server build doesn't move walkers with WalkerControl (map_viewer/test_walker_control.py),
    and moving them with set_location leaves them sliding in a frozen standing pose. A walker
    state puts the walker exactly where asked and plays the walk cycle at the given speed,
    once the walker's animation has been enabled by briefly running an AI walker controller.
    """

    _GROUND_STEP = 0.5  # [m] spacing of the ground height samples along the path

    def __init__(self, walker, spot, yaw, path_length):
        self.walker = walker
        self.yaw = yaw
        self.direction = carla.Rotation(yaw=yaw).get_forward_vector()
        self.placed = False
        self._setup_step = 0
        self._ai_controller = None

        world = CarlaDataProvider.get_world()
        carla_map = CarlaDataProvider.get_map()
        self._ground = []
        for k in range(int(path_length / self._GROUND_STEP) + 2):
            d = k * self._GROUND_STEP
            location = spot + carla.Location(x=self.direction.x * d, y=self.direction.y * d)
            self._ground.append(self._ground_height(world, carla_map, location))
        self.spot = carla.Location(spot.x, spot.y, self._ground[0])

    _GROUND_LABELS = ('Roads', 'RoadLines', 'Sidewalks', 'Ground', 'Terrain')
    _GROUND_TOLERANCE = 0.6  # [m] around the road / sidewalk height given by the map

    @classmethod
    def _ground_height(cls, world, carla_map, location):
        """
        Height of the ground (road, sidewalk, terrain) at a location. The map gives the road /
        sidewalk height there; the actual surface is taken from a vertical ray, only accepting hits
        close to that height (preferring those labelled as ground). This ignores anything above the
        ground, whatever its label: the blocker's top, invisible sign trigger boxes, ... which would
        otherwise leave the pedestrian standing on them or floating in the air.
        """
        reference = carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Any)
        ref_z = reference.transform.location.z if reference else location.z
        hits = world.cast_ray(carla.Location(location.x, location.y, ref_z + 3.0),
                              carla.Location(location.x, location.y, ref_z - 3.0))
        near = [hit for hit in hits if abs(hit.location.z - ref_z) < cls._GROUND_TOLERANCE]
        ground = [hit for hit in near if str(hit.label) in cls._GROUND_LABELS]
        if ground:
            return max(hit.location.z for hit in ground)
        if near:
            return max(hit.location.z for hit in near)
        return ref_z

    def _ground_z(self, distance):
        index = min(distance / self._GROUND_STEP, len(self._ground) - 1.001)
        k = int(index)
        frac = index - k
        return self._ground[k] * (1 - frac) + self._ground[k + 1] * frac

    def set_state(self, distance, speed):
        """Puts the walker at 'distance' along its path, with a walk cycle for 'speed' [m/s]"""
        location = carla.Location(x=self.spot.x + self.direction.x * distance,
                                  y=self.spot.y + self.direction.y * distance,
                                  z=self._ground_z(distance))
        CarlaDataProvider.get_client().apply_batch([carla.command.ApplyWalkerState(
            self.walker.id, carla.Transform(location, carla.Rotation(yaw=self.yaw)), speed)])

    def place(self):
        """
        Puts the walker standing at the start of its path. Needs a few calls, one per tick
        (the AI controller has to be spawned, started and stopped on different ticks).
        Returns True once the walker is in place.
        """
        if self.placed:
            return True
        if self._setup_step == 0:
            world = CarlaDataProvider.get_world()
            self.walker.set_simulate_physics(True)
            self.walker.set_location(self.spot + carla.Location(z=1.0))
            self._ai_controller = world.spawn_actor(
                world.get_blueprint_library().find('controller.ai.walker'), carla.Transform(), self.walker)
        elif self._setup_step == 1:
            self._ai_controller.start()  # snaps the walker to the navmesh, it's placed back below
        elif self._setup_step == 2:
            self._ai_controller.stop()
            self._ai_controller.destroy()
            self._ai_controller = None
        else:
            self.set_state(0, 0)
            self.placed = True
        self._setup_step += 1
        return self.placed


class RevealPedestrian(py_trees.behaviour.Behaviour):
    """
    Keeps the pedestrian hidden under the ego (large map dormancy workaround, see
    MovePedestrianWithEgo) until the ego is within 'reveal_distance' of its spot, then
    puts it standing there. This way it's already present long before the ego is close,
    instead of popping up a few meters in front of it.
    """

    def __init__(self, ego, crossing_walker, reveal_distance, name="RevealPedestrian"):
        super().__init__(name)
        self._ego = ego
        self._walker = crossing_walker
        self._reveal_distance = reveal_distance
        self._hide = MovePedestrianWithEgo(ego, crossing_walker.walker, 100)

    def initialise(self):
        self._hide.initialise()
        super().initialise()

    def update(self):
        if not self._walker.placed:
            ego_location = CarlaDataProvider.get_location(self._ego)
            if ego_location is not None and ego_location.distance(self._walker.spot) > self._reveal_distance:
                self._hide.update()
            else:
                self._walker.place()
        return py_trees.common.Status.RUNNING


class PlacePedestrian(py_trees.behaviour.Behaviour):
    """Makes sure the pedestrian stands at its spot (if it wasn't revealed already)"""

    def __init__(self, crossing_walker, name="PlacePedestrian"):
        super().__init__(name)
        self._walker = crossing_walker

    def update(self):
        if self._walker.place():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WalkerCrossing(py_trees.behaviour.Behaviour):
    """
    Walks the pedestrian along its path from 'start' to 'end' [m], speeding up from
    standstill over 'accel_time' [s], and stops it at the end.
    """

    def __init__(self, crossing_walker, speed, start, end, accel_time=0.6, name="WalkerCrossing"):
        super().__init__(name)
        self._walker = crossing_walker
        self._speed = speed
        self._start = start
        self._end = end
        self._accel = speed / accel_time
        self._distance = start
        self._current_speed = 0.0
        self._last_time = None

    def initialise(self):
        self._distance = self._start
        self._current_speed = 0.0
        self._last_time = GameTime.get_time()
        super().initialise()

    def update(self):
        now = GameTime.get_time()
        delta = now - self._last_time
        self._last_time = now

        self._current_speed = min(self._speed, self._current_speed + self._accel * delta)
        self._distance = min(self._end, self._distance + self._current_speed * delta)
        if self._distance >= self._end:
            self._walker.set_state(self._end, 0)
            return py_trees.common.Status.SUCCESS
        self._walker.set_state(self._distance, self._current_speed)
        return py_trees.common.Status.RUNNING


class StationaryObjectCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist.
    The ego vehicle is passing through a road and encounters
    a stationary cyclist.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 40

        # other vehicle parameters
        self._other_actor_target_velocity = 10
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(StationaryObjectCrossing, self).__init__("Stationaryobjectcrossing",
                                                       ego_vehicles,
                                                       config,
                                                       world,
                                                       debug_mode,
                                                       criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        _distance = 40
        lane_width = self._reference_waypoint.lane_width
        location, _ = get_location_in_distance_from_wp(self._reference_waypoint, _distance)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.4, "k": 0.2}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z += offset['z']
        self.transform = carla.Transform(location, carla.Rotation(yaw=orientation_yaw))
        static = CarlaDataProvider.request_new_actor('static.prop.container', self.transform)
        static.set_simulate_physics(True)
        self.other_actors.append(static)

    def _create_behavior(self):
        """
        Only behavior here is to wait
        """
        lane_width = self.ego_vehicles[0].get_world().get_map().get_waypoint(
            self.ego_vehicles[0].get_location()).lane_width
        lane_width = lane_width + (1.25 * lane_width)

        # leaf nodes
        actor_stand = Idle(15)
        actor_removed = ActorDestroy(self.other_actors[0])
        end_condition = DriveDistance(self.ego_vehicles[0], self._ego_vehicle_distance_driven)

        # non leaf nodes
        root = py_trees.composites.Parallel(
            name="StaticObstacle",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        scenario_sequence = py_trees.composites.Sequence()

        # building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(actor_stand)
        scenario_sequence.add_child(actor_removed)
        scenario_sequence.add_child(end_condition)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class DynamicObjectCrossing(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=60,
    ):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._num_lane_changes = 0

        self._blocker_shift = 0.9
        self._blocker_side_offset = 1.5  # [m] blocker and pedestrian, from the sidewalk's edge waypoint
        self._retry_dist = 0.4

        self._adversary_transform = None
        self._blocker_transform = None
        self._collision_wp = None

        self._adversary_speed = get_value_parameter(config, 'walker_speed', float, 1.6)  # brisk walk [m/s]
        self._crossing_angle = get_value_parameter(config, 'crossing_angle', float, 0)
        self._reaction_time = 2.1  # Time the agent has to react to avoid the collision [s]
        self._reaction_time += 0.1 * floor(self._crossing_angle / 5)
        # Tuned for a 2 m/s adversary: start a slower one earlier so it reaches the lane at the same moment
        self._reaction_time *= 2.0 / self._adversary_speed
        # Distance to the ego at which the pedestrian is put in place, standing on the sidewalk
        self._reveal_distance = get_value_parameter(config, 'reveal_distance', float, 80)
        self._walk_away_distance = 3.0  # [m] walked past the far side of the road before stopping
        self._crossing_walker = None
        self._walker_clearance = 1.0  # [m] between the blocker's far end and the pedestrian's center
        self._min_trigger_dist = 6.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40
        self.timeout = timeout

        self._number_of_attempts = 6

        self._distance = get_value_parameter(config, 'distance', float, 12)
        # The pedestrian waits behind the blocker until crossing: it has to be big enough to hide it
        # (a kiosk, ~2.5 x 2.5 x 3 m, by default). If the model can't be spawned, other big ones are tried
        self._blocker_model = get_value_parameter(config, 'blocker_model', str, 'static.prop.kiosk_01')
        self._blocker_fallbacks = ['static.prop.kiosk_01', 'vehicle.sprinter.mercedes', 'static.prop.container',
                                   'static.prop.advertisement', 'static.prop.vendingmachine']
        if abs(self._crossing_angle) > 90:
            raise ValueError("'crossing_angle' must be between -90 and 90º for the pedestrian to cross the road")
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        super(DynamicObjectCrossing, self).__init__("DynamicObjectCrossing",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    @staticmethod
    def _half_length_along(actor, actor_transform, waypoint):
        """
        Half of the actor's size along the waypoint's direction, from its bounding box and orientation.
        The orientation is the one it was spawned with: right after spawning, get_transform() doesn't
        return it yet.
        """
        extent = actor.bounding_box.extent
        delta = math.radians(actor_transform.rotation.yaw - waypoint.transform.rotation.yaw)
        return abs(extent.x * math.cos(delta)) + abs(extent.y * math.sin(delta))

    @staticmethod
    def _footprint_gap(location, actor, actor_transform):
        """Horizontal distance from a location to the actor's footprint (its oriented bounding box)"""
        box = actor.bounding_box
        center = actor_transform.transform(box.location)
        yaw = math.radians(actor_transform.rotation.yaw)
        dx, dy = location.x - center.x, location.y - center.y
        along = abs(dx * math.cos(yaw) + dy * math.sin(yaw)) - box.extent.x
        across = abs(-dx * math.sin(yaw) + dy * math.cos(yaw)) - box.extent.y
        return math.hypot(max(along, 0.0), max(across, 0.0))

    def _get_sidewalk_transform(self, waypoint, offset):
        """
        Processes the waypoint transform to find a suitable spawning one at the sidewalk.
        It first rotates the transform so that it is pointing towards the road and then moves a
        bit to the side waypoint that aren't part of sidewalks, as they might be invading the road
        """
        if self._direction == "left":
            offset['yaw'] *= -1
            offset['k'] *= -1

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += offset['yaw']

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            right_vector = waypoint.transform.get_right_vector()
            offset_dist = offset["k"]
            offset_location = carla.Location(offset_dist * right_vector.x, offset_dist * right_vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += offset['z']

        return carla.Transform(new_location, new_rotation)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Get the waypoint in front of the ego.
        move_dist = self._distance
        waypoint = self._reference_waypoint

        while self._number_of_attempts > 0:
            parking_location = None
            self._collision_dist = 0

            # Move to the front
            location, _ = get_location_in_distance_from_wp(waypoint, move_dist, False)
            waypoint = self._wmap.get_waypoint(location)
            self._collision_wp = waypoint

            # Move to the right
            sidewalk_waypoint = waypoint
            while sidewalk_waypoint.lane_type != carla.LaneType.Sidewalk:
                if self._direction == "right":
                    side_wp = sidewalk_waypoint.get_right_lane()
                else:
                    side_wp = sidewalk_waypoint.get_left_lane()
                if side_wp is None:
                    break  # No more side lanes
                sidewalk_waypoint = side_wp
                if side_wp.lane_type == carla.LaneType.Parking:
                    parking_location = side_wp.transform.location

            # Get the blocker transform and spawn it (falling back to other big models)
            blocker = None
            library = CarlaDataProvider.get_world().get_blueprint_library()
            for model in [self._blocker_model] + [m for m in self._blocker_fallbacks if m != self._blocker_model]:
                # Models missing from this build would silently be replaced by a small default prop
                if not [bp for bp in library.filter(model) if not bp.id.endswith('.vr')]:
                    continue
                offset = {"yaw": 0 if 'vehicle' in model else 90, "z": 0.0, "k": self._blocker_side_offset}
                self._blocker_transform = self._get_sidewalk_transform(sidewalk_waypoint, offset)
                blocker = CarlaDataProvider.request_new_actor(
                    model, self._blocker_transform, rolename="scenario no lights")
                if blocker:
                    if model != self._blocker_model:
                        print("DynamicObjectCrossing: blocker {} couldn't be spawned, using {}".format(
                            self._blocker_model, model))
                    break
            if not blocker:
                self._number_of_attempts -= 1
                move_dist = self._retry_dist
                print("Failed to spawn the blocker")
                continue

            # Get the adversary transform and spawn it, behind the blocker (as seen from the ego): past
            # its far end along the sidewalk, measured from its actual size and spawn orientation (it's
            # turned across the road, and the model may be a fallback one with other dimensions). Then
            # make sure the spot is clear of the blocker's footprint, moving further along if needed
            walker_dist = self._half_length_along(blocker, self._blocker_transform, sidewalk_waypoint) \
                + self._walker_clearance
            walker_wp = None
            for _ in range(12):
                wps = sidewalk_waypoint.next(walker_dist)
                if not wps:
                    break
                walker_wp = wps[0]
                # Same side offset as the blocker: right behind it as seen from the road
                offset = {"yaw": 270 - self._crossing_angle, "z": 1.2, "k": self._blocker_side_offset}
                self._adversary_transform = self._get_sidewalk_transform(walker_wp, offset)
                gap = self._footprint_gap(self._adversary_transform.location, blocker, self._blocker_transform)
                if gap >= self._walker_clearance:
                    break
                walker_dist += 0.5
            if walker_wp is None:
                raise ValueError("Couldn't find a location to spawn the adversary")
            print("DynamicObjectCrossing: blocker {} at {} yaw {:.0f} extent ({:.2f}, {:.2f}); pedestrian "
                  "{:.1f} m further along the sidewalk at {}, {:.2f} m from the blocker's footprint".format(
                      blocker.type_id, self._blocker_transform.location, self._blocker_transform.rotation.yaw,
                      blocker.bounding_box.extent.x, blocker.bounding_box.extent.y, walker_dist,
                      self._adversary_transform.location, gap))

            adversary = CarlaDataProvider.request_new_actor('walker.*', self._adversary_transform)
            if adversary is None:
                CarlaDataProvider.remove_actor_by_id(blocker.id)
                self._number_of_attempts -= 1
                move_dist = self._retry_dist
                print("Failed to spawn an adversary")
                continue

            self._collision_dist += waypoint.transform.location.distance(self._adversary_transform.location)

            # Both actors were succesfully spawned, end
            break

        if self._number_of_attempts == 0:
            raise Exception("Couldn't find viable position for the adversary and blocker actors")

        blocker.set_simulate_physics(False)
        adversary.set_location(self._adversary_transform.location + carla.Location(z=-200))
        adversary = self._replace_walker(adversary)
        self._crossing_walker = CrossingWalker(
            adversary, self._adversary_transform.location, self._adversary_transform.rotation.yaw,
            2 * self._collision_dist + self._walk_away_distance)

        if parking_location:
            self.parking_slots.append(parking_location)

        self.other_actors.append(adversary)
        self.other_actors.append(blocker)

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence(name="CrossingActor")
        if self.route_mode:
            total_dist = self._distance + 10
            sequence.add_child(LeaveSpaceInFront(total_dist))

        # Normally already standing there since the ego got within the reveal distance
        sequence.add_child(PlacePedestrian(self._crossing_walker))
        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the adversary: cross the whole road (supposing symetry in both directions)
        # and a few steps more, then stand there
        move_distance = 2 * self._collision_dist + self._walk_away_distance
        if self.route_mode:
            sequence.add_child(LeaveCrossingSpace(self._collision_wp))
        sequence.add_child(WalkerCrossing(
            self._crossing_walker, self._adversary_speed, 0, move_distance, name="AdversaryCrossing"))

        # Remove everything once the ego has driven away, instead of making the pedestrian vanish
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))
        sequence.add_child(ActorDestroy(self.other_actors[0], name="DestroyAdversary"))
        sequence.add_child(ActorDestroy(self.other_actors[1], name="DestroyBlocker"))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

    # TODO: Pedestrian have an issue with large maps were setting them to dormant breaks them,
    # so all functions below are meant to patch it until the fix is done
    def _replace_walker(self, adversary):
        """As the adversary is probably, replace it with another one"""
        type_id = adversary.type_id
        CarlaDataProvider.remove_actor_by_id(adversary.id)
        spawn_transform = self.ego_vehicles[0].get_transform()
        spawn_transform.location.z -= 50
        adversary = CarlaDataProvider.request_new_actor(type_id, spawn_transform)
        if not adversary:
            raise ValueError("Couldn't spawn the walker substitute")
        adversary.set_simulate_physics(False)
        adversary.set_location(spawn_transform.location + carla.Location(z=-50))
        return adversary

    def _setup_scenario_trigger(self, config):
        """Normal scenario trigger but in parallel, a behavior that ensures the pedestrian stays active"""
        trigger_tree = super()._setup_scenario_trigger(config)

        if not self.route_mode:
            return trigger_tree

        parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="ScenarioTrigger")

        parallel.add_child(RevealPedestrian(self.ego_vehicles[0], self._crossing_walker, self._reveal_distance))

        parallel.add_child(trigger_tree)
        return parallel


class ParkingCrossingPedestrian(BasicScenario):

    """
    Variation of DynamicObjectCrossing but now the blocker is now a vehicle
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=60,
    ):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()
        self._trigger_location = config.trigger_points[0].location
        self._reference_waypoint = self._wmap.get_waypoint(self._trigger_location)
        self._num_lane_changes = 0

        self._adversary_speed = 2.0  # Speed of the adversary [m/s]
        self._min_trigger_dist = 6.0  # Min distance to the collision location that triggers the adversary [m]
        self._ego_end_distance = 40
        self.timeout = timeout

        self._bp_attributes = {'base_type': 'car', 'generation': 2}

        self._distance = get_value_parameter(config, 'distance', float, 12)
        self._crossing_angle = get_value_parameter(config, 'crossing_angle', float, 0)
        if abs(self._crossing_angle) > 90:
            raise ValueError("'crossing_angle' must be between -90 and 90º for the pedestrian to cross the road")
        self._direction = get_value_parameter(config, 'direction', str, 'right')
        if self._direction not in ('left', 'right'):
            raise ValueError(f"'direction' must be either 'right' or 'left' but {self._direction} was given")

        # Time the agent has to react to avoid the collision [s]
        self._reaction_time = 2.15
        self._reaction_time += 0.1 * floor(self._crossing_angle / 5)

        super().__init__("ParkingCrossingPedestrian",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _get_blocker_transform(self, waypoint):
        """Processes the driving wp to get a waypoint at the side that looks at the road"""
        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            vector = waypoint.transform.get_right_vector()
            if self._direction == 'left':
                vector *= -1

            offset_location = carla.Location(waypoint.lane_width * vector.x, waypoint.lane_width * vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += 0.5

        return carla.Transform(new_location, waypoint.transform.rotation)

    def _get_walker_transform(self, waypoint):
        """Processes the driving wp to get a waypoint at the side that looks at the road"""

        new_rotation = waypoint.transform.rotation
        new_rotation.yaw += 270 - self._crossing_angle if self._direction == 'right' else 90 + self._crossing_angle

        if waypoint.lane_type == carla.LaneType.Sidewalk:
            new_location = waypoint.transform.location
        else:
            vector = waypoint.transform.get_right_vector()
            if self._direction == 'left':
                vector *= -1

            offset_location = carla.Location(waypoint.lane_width * vector.x, waypoint.lane_width * vector.y)
            new_location = waypoint.transform.location + offset_location
        new_location.z += 1.2

        return carla.Transform(new_location, new_rotation)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # Get the adversary transform and spawn it
        wps = self._reference_waypoint.next(self._distance)
        if not wps:
            raise ValueError("Couldn't find a location to spawn the adversary")
        blocker_wp = wps[0]

        # Get the adversary transform and spawn it
        self._blocker_transform = self._get_blocker_transform(blocker_wp)
        self.parking_slots.append(self._blocker_transform.location)
        blocker = CarlaDataProvider.request_new_actor(
            'vehicle.*', self._blocker_transform, attribute_filter=self._bp_attributes)
        if blocker is None:
            raise ValueError("Couldn't spawn the adversary")
        self.other_actors.append(blocker)
        blocker.apply_control(carla.VehicleControl(hand_brake=True))

        walker_dist = blocker.bounding_box.extent.x + 0.5
        wps = blocker_wp.next(walker_dist)
        if not wps:
            raise ValueError("Couldn't find a location to spawn the adversary")
        walker_wp = wps[0]

        # Get the adversary transform and spawn it
        self._walker_transform = self._get_walker_transform(walker_wp)
        self.parking_slots.append(self._walker_transform.location)

        walker = CarlaDataProvider.request_new_actor('walker.*', self._walker_transform)
        if walker is None:
            raise ValueError("Couldn't spawn the adversary")

        walker.set_location(self._walker_transform.location + carla.Location(z=-200))
        walker = self._replace_walker(walker)

        self.other_actors.append(walker)

        self._collision_wp = walker_wp

    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """
        sequence = py_trees.composites.Sequence(name="ParkingCrossingPedestrian")
        if self.route_mode:
            total_dist = self._distance + 15
            sequence.add_child(LeaveSpaceInFront(total_dist))

        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._walker_transform, True))
        collision_location = self._collision_wp.transform.location

        # Wait until ego is close to the adversary
        trigger_adversary = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="TriggerAdversaryStart")
        trigger_adversary.add_child(InTimeToArrivalToLocation(
            self.ego_vehicles[0], self._reaction_time, collision_location))
        trigger_adversary.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], collision_location, self._min_trigger_dist))
        sequence.add_child(trigger_adversary)

        # Move the adversary
        distance = 8.0  # Scenario is meant to be used at a one lane - one direction road
        duration = distance / self._adversary_speed

        sequence.add_child(KeepVelocity(
            self.other_actors[1], self._adversary_speed,
            duration=duration, distance=distance, name="AdversaryCrossing"))

        # Remove everything
        sequence.add_child(ActorDestroy(self.other_actors[1], name="DestroyAdversary"))
        sequence.add_child(ActorDestroy(self.other_actors[0], name="DestroyBlocker"))
        sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_end_distance, name="EndCondition"))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        if self.route_mode:
            return []
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

    # TODO: Pedestrian have an issue with large maps were setting them to dormant breaks them,
    # so all functions below are meant to patch it until the fix is done
    def _replace_walker(self, walker):
        """As the adversary is probably, replace it with another one"""
        type_id = walker.type_id
        CarlaDataProvider.remove_actor_by_id(walker.id)
        spawn_transform = self.ego_vehicles[0].get_transform()
        spawn_transform.location.z -= 50
        walker = CarlaDataProvider.request_new_actor(type_id, spawn_transform)
        if not walker:
            raise ValueError("Couldn't spawn the walker substitute")
        walker.set_simulate_physics(False)
        walker.set_location(spawn_transform.location + carla.Location(z=-50))
        return walker

    def _setup_scenario_trigger(self, config):
        """Normal scenario trigger but in parallel, a behavior that ensures the pedestrian stays active"""
        trigger_tree = super()._setup_scenario_trigger(config)

        if not self.route_mode:
            return trigger_tree

        parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="ScenarioTrigger")

        parallel.add_child(MovePedestrianWithEgo(self.ego_vehicles[0], self.other_actors[1], 100))

        parallel.add_child(trigger_tree)
        return parallel
