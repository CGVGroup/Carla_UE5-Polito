import time
import carla
import argparse
import logging
import random

def spawn_pedestrian(world, position):
    """Spawn a pedestrian at a given position with an obstacle detector."""
    blueprint = world.get_blueprint_library().filter('walker.pedestrian.*')[0]
    transform = carla.Transform(carla.Location(*position))
    walker = world.try_spawn_actor(blueprint, transform)
    if walker:
        attach_obstacle_sensor(walker, world)
    return walker

def attach_obstacle_sensor(walker, world):
    """Attach an obstacle detector sensor to the pedestrian."""
    blueprint = world.get_blueprint_library().find('sensor.other.obstacle')
    blueprint.set_attribute('distance', '10.0')  # Detection range in meters
    blueprint.set_attribute('only_dynamics', 'true')  # Detect only dynamic objects
    sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=walker)
    sensor.listen(lambda event: on_obstacle_detected(walker, event))

def on_obstacle_detected(walker, event):
    """Trigger pedestrian to run across the road when an obstacle is detected."""
    print("Obstacle detected! Pedestrian starts crossing.")
    walker_location = walker.get_location()
    destination = walker_location + carla.Location(x=5)  # Adjust x offset as needed
    walker_control = walker.get_control()
    walker_control.speed = 3.0  # Running speed
    walker_control.direction = (destination - walker_location).make_unit_vector()
    walker.apply_control(walker_control)

def main():
    argparser = argparse.ArgumentParser(description='Spawn a pedestrian that crosses when an obstacle is detected.')
    argparser.add_argument('--mode', choices=['HUD', 'HDD'], required=True, help='Select pedestrian spawn mode: HUD or HDD')
    args = argparser.parse_args()
    
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    # Hardcoded values based on mode
    spawn_positions = {
        'HUD': (10.0, 5.0, 0.0),  # Example position for HUD mode
        'HDD': (20.0, 10.0, 0.0)  # Example position for HDD mode
    }

    try:
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False  # Enable asynchronous mode
        world.apply_settings(settings)
        
        walker = spawn_pedestrian(world, spawn_positions[args.mode])
        
        if walker is None:
            print("Failed to spawn walker.")
            return
        
        print("Pedestrian spawned with obstacle detector attached.")
        
        while True:
            time.sleep(1)  # Prevents busy looping while keeping async behavior
        
    finally:
        if walker:
            walker.destroy()
        print("Done.")

if __name__ == '__main__':
    main()
