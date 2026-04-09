import time
import carla
import argparse
import logging
import random

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)
    if generation.lower() == "all":
        return bps
    if len(bps) == 1:
        return bps
    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            return [x for x in bps if int(x.get_attribute('generation')) == int_generation]
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def get_deterministic_spawn_points(seed, world, num_points):
    """Generate a deterministic list of spawn points based on a fixed seed."""
    random.seed(seed)
    spawn_points = world.get_map().get_spawn_points()
    spawn_points = sorted(spawn_points, key=lambda sp: (sp.location.x, sp.location.y, sp.location.z))
    return spawn_points[:num_points] if num_points <= len(spawn_points) else spawn_points

def main():
    argparser = argparse.ArgumentParser(description='Generate deterministic traffic in CARLA.')
    argparser.add_argument('--host', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('--port', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--seed', type=int, required=True, help='Fixed seed for deterministic behavior')
    argparser.add_argument('-n', '--number-of-vehicles', default=30, type=int, help='Number of vehicles to spawn (default: 30)')
    argparser.add_argument('-w', '--number-of-walkers', default=10, type=int, help='Number of walkers to spawn (default: 10)')
    argparser.add_argument('--filterv', default='vehicle.*', help='Filter for selecting vehicle blueprints (default: "vehicle.*")')
    argparser.add_argument('--generationv', default='All', help='Vehicle generation filter (default: "All")')
    argparser.add_argument('--filterw', default='walker.pedestrian.*', help='Filter for selecting pedestrian blueprints (default: "walker.pedestrian.*")')
    argparser.add_argument('--generationw', default='All', help='Pedestrian generation filter (default: "All")')
    args = argparser.parse_args()
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_random_device_seed(args.seed)
        
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        blueprints_walkers = get_actor_blueprints(world, args.filterw, args.generationw)
        
        spawn_points = get_deterministic_spawn_points(args.seed, world, args.number_of_vehicles)
        
        batch = []
        for i in range(args.number_of_vehicles):
            blueprint = blueprints[i % len(blueprints)]
            transform = spawn_points[i]
            batch.append(carla.command.SpawnActor(blueprint, transform)
                         .then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())))
        
        responses = client.apply_batch_sync(batch, True)
        vehicle_ids = [r.actor_id for r in responses if not r.error]
        
        walker_batch = []
        walker_speed = []
        walker_spawn_points = get_deterministic_spawn_points(args.seed + 1, world, args.number_of_walkers)
        for i in range(args.number_of_walkers):
            walker_bp = blueprints_walkers[i % len(blueprints_walkers)]
            transform = walker_spawn_points[i]
            walker_batch.append(carla.command.SpawnActor(walker_bp, transform))
            walker_speed.append(1.4)
        
        responses = client.apply_batch_sync(walker_batch, True)
        walker_ids = [r.actor_id for r in responses if not r.error]
        
        print(f'Spawned {len(vehicle_ids)} vehicles and {len(walker_ids)} walkers deterministically.')
        
        while True:
            world.tick()
    
    finally:
        print('Cleaning up actors...')
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_ids + walker_ids])
        world.apply_settings(settings)
        print('Done.')

if __name__ == '__main__':
    main()
