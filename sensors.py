import carla

# Connect to the Carla server
client = carla.Client('localhost', 3000)
client.set_timeout(10.0)

# Get the world object
world = client.get_world()

sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
# Find all traffic lights in the world
traffic_lights = world.get_actors().filter('traffic.traffic_light')
print(traffic_lights)
# Attach a collision sensor to each traffic light
# for traffic_light in traffic_lights:
#     sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
#     sensor_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
#     sensor = world.spawn_actor(sensor_bp, sensor_transform, attach_to=traffic_light)

#     # Define the callback function to print a message when a collision is detected
#     def on_collision(event):
#         print('Collision detected with', event.other_actor)

#     # Bind the callback function to the collision sensor
#     sensor.listen(lambda event: on_collision(event))