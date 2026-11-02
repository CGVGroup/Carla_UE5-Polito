#!/usr/bin/env python3
"""
measure_steering.py: measures a vehicle's steering response (steer -> path curvature) for the pure pursuit
steering of the leaderboard's NPC agent (BasicAgent2), and prints the line to add to its
STEERING_CALIBRATIONS (PythonAPI/carla/agents/navigation/basic_agent2.py).

For each steer value, to the left and to the right: the vehicle is put back at the start point, driven
straight up to the measurement speed, then the steer is held at that speed and, once the turn has settled,
the curvature is measured (yaw rate / speed). Runs in synchronous mode, needs ~20 m of free, flat space
around the start point (a wide junction or a square): pass it with --x --y --yaw (default: the first
spawn point of the map). Run it on an idle server (no leaderboard / traffic running).

Usage:
    python measure_steering.py --model vehicle.mercedes.vr --x 120 --y -40 --yaw 90
"""
import argparse
import math
import sys

import carla


def wheelbase_of(vehicle):
    wheels = vehicle.get_physics_control().wheels
    front = [w for w in wheels if w.max_steer_angle > 0.1] or wheels[:2]
    rear = [w for w in wheels if w not in front] or wheels[2:]

    def center(ws):
        return carla.Location(x=sum(w.position.x for w in ws) / len(ws), y=sum(w.position.y for w in ws) / len(ws),
                              z=sum(w.position.z for w in ws) / len(ws))
    distance = center(front).distance(center(rear))
    return (distance / 100.0 if distance > 20.0 else distance), max(w.max_steer_angle for w in front)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--model', default='vehicle.mercedes.vr', help='vehicle blueprint')
    ap.add_argument('--x', type=float, help='start point (default: first spawn point)')
    ap.add_argument('--y', type=float)
    ap.add_argument('--z', type=float, help='default: the road height there')
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--speed', type=float, default=11.0, help='measurement speed [km/h] (BasicAgent2: ~11)')
    ap.add_argument('--steers', default='0.2,0.4,0.6,0.8,1.0')
    ap.add_argument('--settle', type=float, default=1.5, help='[s] of turn before measuring')
    ap.add_argument('--measure', type=float, default=1.5, help='[s] of measurement')
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    carla_map = world.get_map()

    if args.x is not None and args.y is not None:
        location = carla.Location(args.x, args.y, 0.0)
        waypoint = carla_map.get_waypoint(location)
        location.z = (args.z if args.z is not None else waypoint.transform.location.z) + 0.3
        start = carla.Transform(location, carla.Rotation(yaw=args.yaw))
    else:
        start = carla_map.get_spawn_points()[0]
        start.location.z += 0.3
    print('Start point:', start)

    blueprints = world.get_blueprint_library().filter(args.model)
    if not blueprints:
        print('Blueprint {} not found'.format(args.model))
        return 1
    blueprint = blueprints[0]
    if blueprint.has_attribute('role_name'):
        blueprint.set_attribute('role_name', 'calibration')

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    dt = settings.fixed_delta_seconds

    vehicle = None
    collisions = []
    sensor = None
    try:
        vehicle = world.spawn_actor(blueprint, start)
        world.tick()
        sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),
                                   carla.Transform(), attach_to=vehicle)
        sensor.listen(lambda event: collisions.append(event.other_actor.type_id if event.other_actor else '?'))
        wheelbase, max_angle = wheelbase_of(vehicle)
        print('{}: wheelbase {:.3f} m, max wheel angle {:.1f} deg (physics)'.format(vehicle.type_id, wheelbase, max_angle))

        target = args.speed / 3.6
        integral = 0.0

        def drive(steer, seconds, record=False):
            """Holds the speed with a PI throttle, with a fixed steer. Returns the curvatures measured"""
            nonlocal integral
            samples = []
            for _ in range(int(seconds / dt)):
                v = vehicle.get_velocity()
                speed = math.hypot(v.x, v.y)
                error = target - speed
                integral = max(-2.0, min(2.0, integral + error * dt))
                throttle = max(0.0, min(1.0, 0.35 * error + 0.25 * integral + 0.15))
                brake = 0.0 if error > -0.5 else min(1.0, -0.3 * error)
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))
                world.tick()
                if record and speed > 0.5 * target:
                    yaw_rate = math.radians(vehicle.get_angular_velocity().z)
                    samples.append(abs(yaw_rate) / speed)
            return samples

        results = [(0.0, 0.0)]
        for steer in [float(s) for s in args.steers.split(',')]:
            curvatures = []
            for sign in (1.0, -1.0):
                vehicle.set_target_velocity(carla.Vector3D())
                vehicle.set_target_angular_velocity(carla.Vector3D())
                vehicle.set_transform(start)
                vehicle.apply_control(carla.VehicleControl(hand_brake=True))
                for _ in range(10):
                    world.tick()
                integral = 0.0
                collisions.clear()
                drive(0.0, 3.0)  # straight, up to speed
                drive(sign * steer, args.settle)
                samples = drive(sign * steer, args.measure, record=True)
                if collisions:
                    print('  steer {:+.2f}: collision with {} - pick a freer start point'.format(sign * steer, collisions[0]))
                if samples:
                    curvature = sum(samples) / len(samples)
                    curvatures.append(curvature)
                    print('  steer {:+.2f}: curvature {:.4f} 1/m (radius {:.1f} m)'.format(
                        sign * steer, curvature, 1.0 / curvature if curvature > 1e-6 else float('inf')))
            if curvatures:
                results.append((steer, sum(curvatures) / len(curvatures)))

        line = "    '{}': ({:.2f}, [{}]),".format(
            vehicle.type_id.lower(), wheelbase, ', '.join('({:.1f}, {:.4f})'.format(s, k) for s, k in results))
        print('\nAdd to STEERING_CALIBRATIONS in PythonAPI/carla/agents/navigation/basic_agent2.py:\n')
        print(line)
    finally:
        if sensor is not None:
            sensor.stop()
            sensor.destroy()
        if vehicle is not None:
            vehicle.destroy()
        world.apply_settings(original)
    return 0


if __name__ == '__main__':
    sys.exit(main())
