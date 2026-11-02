#!/usr/bin/env python3
"""
reset_world.py — Force a CARLA server back to asynchronous mode and destroy
all vehicles and scenario-spawned static objects left in the world.

If run_leaderboard_xai.ps1 gets killed abruptly (closed terminal window,
forced process kill, etc.) instead of exiting cleanly, the server can be
left stuck in synchronous mode with no client ticking it (looks like a
frozen server), and with all the run's vehicles (ego, background traffic,
scenario actors) and static props (construction cones, warning signs,
debris, crossing blockers, ...) still in the world. Run this to un-stick
it and clear those out, without relaunching the whole CARLA server.

Static objects are only destroyed if their rolename starts with "scenario"
(how CarlaDataProvider.request_new_actor tags everything scenario_runner
spawns) - anything actually authored as part of the map itself is left
alone. Walkers and their AI controllers are destroyed with the vehicles.

Then the map is reloaded (unless --no-reload): the VR vehicles' Blueprints
spawn parts that are not CARLA actors (driving seat and steering wheel, VR
pawn, AR-HUD boxes, virtual carpets, dashboard text...), which can't be
listed nor destroyed through the Python API and would otherwise stay in the
world. Reloading the map clears them all.

Usage:
    python reset_world.py
    python reset_world.py --host 127.0.0.1 --port 2000
    python reset_world.py --keep-vehicles         # don't destroy vehicles
    python reset_world.py --keep-static-objects   # don't destroy static props
    python reset_world.py --no-reload             # don't reload the map
"""

import argparse
import time

import carla


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='CARLA server host (default 127.0.0.1)')
    ap.add_argument('--port', type=int, default=2000, help='CARLA server RPC port (default 2000)')
    ap.add_argument('--traffic-manager-port', type=int, default=8000,
                     help='Traffic manager port (default 8000)')
    ap.add_argument('--timeout', type=float, default=10.0, help='Client connection timeout in seconds (default 10)')
    ap.add_argument('--keep-vehicles', action='store_true',
                     help="Don't destroy any vehicles, only reset synchronous mode")
    ap.add_argument('--keep-static-objects', action='store_true',
                     help="Don't destroy scenario-spawned static objects (cones, signs, debris, blockers, ...)")
    ap.add_argument('--no-reload', action='store_true',
                     help="Don't reload the map at the end (parts spawned by the VR vehicles' Blueprints, "
                          "like the steering wheel, then stay in the world)")
    ap.add_argument('--reload-timeout', type=float, default=120.0,
                     help='Timeout for reloading the map, in seconds (default 120)')
    args = ap.parse_args()

    print(f'Connecting to CARLA at {args.host}:{args.port} ...')
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()

    settings = world.get_settings()
    print(f'Current synchronous_mode = {settings.synchronous_mode}')

    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    settings.deterministic_ragdolls = False
    world.apply_settings(settings)
    print('Set synchronous_mode = False')

    try:
        traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
        traffic_manager.set_synchronous_mode(False)
        print('Traffic manager set back to asynchronous mode')
    except RuntimeError as e:
        print(f'Could not reach traffic manager on port {args.traffic_manager_port}: {e}')

    if not args.keep_vehicles:
        # Walkers and their AI controllers (controllers first)
        controllers = world.get_actors().filter('controller.ai.walker')
        for controller in controllers:
            try:
                controller.stop()
            except RuntimeError:
                pass
        walkers = world.get_actors().filter('walker.*')
        client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in list(controllers) + list(walkers)])
        print(f'Destroyed {len(walkers)} walker(s) and {len(controllers)} walker controller(s)')

        vehicles = world.get_actors().filter('vehicle.*')
        sensors = world.get_actors().filter('sensor.*')

        # Destroy sensors attached to a vehicle first, so they don't linger as orphans.
        vehicle_ids = {v.id for v in vehicles}
        vehicle_sensors = [s for s in sensors if s.parent is not None and s.parent.id in vehicle_ids]
        for sensor in vehicle_sensors:
            sensor.stop()
            sensor.destroy()
        if vehicle_sensors:
            print(f'Destroyed {len(vehicle_sensors)} sensor(s) attached to vehicles')

        for vehicle in vehicles:
            vehicle.destroy()
        print(f'Destroyed {len(vehicles)} vehicle(s)')

    if not args.keep_static_objects:
        static_actors = world.get_actors().filter('static.*')
        scenario_statics = [a for a in static_actors
                             if a.attributes.get('role_name', '').startswith('scenario')]
        for actor in scenario_statics:
            actor.destroy()
        print(f'Destroyed {len(scenario_statics)} scenario-spawned static object(s) '
              f'(left {len(static_actors) - len(scenario_statics)} map-authored static actor(s) untouched)')

    if not args.no_reload:
        map_name = world.get_map().name.split('/')[-1]
        print(f'Reloading the map {map_name} (clears what the VR Blueprints spawned: steering wheel, '
              f'driving seat, AR-HUD elements...) ...')
        client.set_timeout(args.reload_timeout)
        try:
            world = client.reload_world(reset_settings=False)
        except RuntimeError as e:
            print(f'reload_world failed ({e}), loading {map_name} instead')
            world = client.load_world(map_name, reset_settings=False)
        # Let the new episode start before handing the server back
        for _ in range(20):
            try:
                world.wait_for_tick(seconds=args.reload_timeout)
                break
            except RuntimeError:
                time.sleep(0.5)
        print('Map reloaded')

    print('Done. The server should respond normally again.')


if __name__ == '__main__':
    main()
