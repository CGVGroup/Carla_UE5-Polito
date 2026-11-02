# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math
import numpy as np
import carla
from agents.tools.misc import get_speed


class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """


    def __init__(self, vehicle, args_lateral, args_longitudinal, offset=0, max_throttle=0.75, max_brake=0.3,
                 max_steering=0.8):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param offset: If different than zero, the vehicle will drive displaced from the center line.
        Positive values imply a right offset while negative ones mean a left one. Numbers high enough
        to cause the vehicle to drive through other lanes might break the controller.
        """

        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self.past_steering = self._vehicle.get_control().steer
        self._lon_controller = PIDLongitudinalController(self._vehicle, **args_longitudinal)
        if args_lateral.get('type') == 'pure_pursuit':
            lateral_args = {k: v for k, v in args_lateral.items() if k != 'type'}
            self._lat_controller = PurePursuitLateralController(self._vehicle, offset, **lateral_args)
        else:
            self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: distance (in meters) to the waypoint
        """

        acceleration = self._lon_controller.run_step(target_speed)
        current_steering = self._lat_controller.run_step(waypoint)
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.

        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering

        return control


    def change_longitudinal_PID(self, args_longitudinal):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller.change_parameters(**args_longitudinal)

    def change_lateral_PID(self, args_lateral):
        """Changes the parameters of the PIDLateralController"""
        self._lat_controller.change_parameters(**args_lateral)

    def set_offset(self, offset):
        """Changes the offset"""
        self._lat_controller.set_offset(offset)


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        # Errors of the last 0.5 s (10 steps at 20 Hz), whatever the simulation step
        self._error_buffer = deque(maxlen=max(2, int(round(0.5 / dt))))

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line. If might cause issues if the value
                is large enough to make the vehicle invade other lanes.
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._offset = offset
        # Errors of the last 0.5 s (10 steps at 20 Hz), whatever the simulation step
        self._e_buffer = deque(maxlen=max(2, int(round(0.5 / dt))))

    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

            :param waypoint: target waypoint
            :return: steering control in the range [-1, 1] where:
            -1 maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def set_offset(self, offset):
        """Changes the offset"""
        self._offset = offset

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

            :param waypoint: target waypoint
            :param vehicle_transform: current transform of the vehicle
            :return: steering control in the range [-1, 1]
        """
        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Get the vector vehicle-target_wp
        if self._offset != 0:
            # Displace the wp to the side
            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
                                                         y=self._offset*r_vec.y)
        else:
            w_loc = waypoint.transform.location

        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class PurePursuitLateralController():
    """
    PurePursuitLateralController steers along the circular arc that joins the rear axle
    to the target waypoint, and converts that arc's curvature into a steering command
    through a measured steer -> curvature map of the vehicle. Unlike the PID controller,
    it needs no steady-state heading error to hold a curve, so it doesn't drift to the
    outside of turns, and the map compensates for vehicles whose steering response is
    far from linear.
    """

    def __init__(self, vehicle, offset=0, wheelbase=2.9, steer_curvature_map=None):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line (positive to the right)
            :param wheelbase: distance between the front and rear axles (m)
            :param steer_curvature_map: list of (steer, curvature in 1/m) pairs measured at
                low speed, with steer increasing from 0. Defaults to an ideal bicycle model
                with a 37 degree maximum wheel angle.
        """
        self._vehicle = vehicle
        self._offset = offset
        self._wheelbase = wheelbase
        if steer_curvature_map is None:
            steer_curvature_map = [(s, math.tan(math.radians(37.0 * s)) / wheelbase)
                                   for s in np.linspace(0.0, 1.0, 11)]
        self._map_steer = np.array([s for s, _ in steer_curvature_map])
        self._map_curvature = np.array([k for _, k in steer_curvature_map])
        self.last_curvature = 0.0  # curvature (1/m) requested on the last step

    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoint.

            :param waypoint: target waypoint
            :return: steering control in the range [-1, 1] (positive to the right)
        """
        vehicle_transform = self._vehicle.get_transform()
        f_vec = vehicle_transform.get_forward_vector()
        f_vec = np.array([f_vec.x, f_vec.y])
        norm = np.linalg.norm(f_vec)
        if norm == 0:
            return 0.0
        f_vec /= norm

        w_loc = waypoint.transform.location
        if self._offset != 0:
            r_vec = waypoint.transform.get_right_vector()
            w_loc = w_loc + carla.Location(x=self._offset * r_vec.x, y=self._offset * r_vec.y)

        # Pure pursuit is defined from the rear axle
        ego_loc = vehicle_transform.location
        rear = np.array([ego_loc.x, ego_loc.y]) - f_vec * (self._wheelbase / 2.0)
        w_vec = np.array([w_loc.x, w_loc.y]) - rear
        lookahead = np.linalg.norm(w_vec)
        if lookahead < 1e-3:
            return 0.0

        # Signed sine of the angle between heading and target (positive = target on the right)
        sin_alpha = (f_vec[0] * w_vec[1] - f_vec[1] * w_vec[0]) / lookahead
        curvature = 2.0 * sin_alpha / lookahead
        self.last_curvature = curvature

        steer = np.interp(abs(curvature), self._map_curvature, self._map_steer)
        return float(np.clip(math.copysign(steer, curvature), -1.0, 1.0))

    def set_offset(self, offset):
        """Changes the offset"""
        self._offset = offset
