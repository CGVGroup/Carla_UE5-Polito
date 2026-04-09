import carla, time
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(5.0)
world = client.get_world()
actors = world.get_actors().filter('vehicle.*')
print(f"Found {len(actors)} vehicles:")
for a in actors:
    print(f"  id={a.id}  type={a.type_id}  attrs={dict(a.attributes)}")