import carla
import random

import time
try:
    client = carla.Client('localhost', 3000)
    client.set_timeout(30.0)

    world = client.get_world()
    print("connected")
    crosswalk_bp = world.get_blueprint_library().find('static.prop.streetsign')
    print(crosswalk_bp)
    # Define crosswalk location and rotation
    start_loc = carla.Location(x=50, y=50, z=0.1)
    start_rot = carla.Rotation(yaw=random.uniform(0, 360))

    # Spawn crosswalk actor
    crosswalk = world.spawn_actor(crosswalk_bp, carla.Transform(start_loc, start_rot))


    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

    # Set camera location and rotation
    camera_loc = carla.Location(x=60, y=60, z=20)
    camera_rot = carla.Rotation(pitch=-30)

    # Spawn camera actor
    camera = world.spawn_actor(camera_bp, carla.Transform(camera_loc, camera_rot))

    # Set camera's sensor parameters
    camera_settings = {
        'PostProcessing': 'SceneFinal',
        'MotionBlur': 'Off',
        'FOV': '90'
    }
    camera_sensor = camera.sensor
    camera_sensor.set(**camera_settings)

    # Define function to set camera transform
    def set_camera_transform(sensor):
        transform = sensor.get_transform()
        transform.location = carla.Location(x=60, y=60, z=20)
        transform.rotation = carla.Rotation(pitch=-30)
        sensor.get_parent().set_transform(transform)

    # Define function to process sensor data
    def process_image(image):
        # Process image here
        pass

    # Listen to camera sensor data
    camera_sensor.listen(lambda image: process_image(image))

    # Set camera transform initially
    set_camera_transform(camera_sensor)
    # Wait for a while
    world.wait_for_tick()
    
finally:
    print("done")
#     walker_bp = world.get_blueprint_library().filter("walker.pedestrian.*")
#     controller_bp = world.get_blueprint_library().find('controller.ai.walker')
#     print(walker_bp)
#     actors = []
#     spawn_points = []
#     for i in range(200):
#         trans = carla.Transform()
#         trans.location = world.get_random_location_from_navigation()
#         trans.location.z +=1
#         spawn_points.append(trans)
#         print(world.get_random_location_from_navigation())
#         walker = random.choice(walker_bp)
#         actor = world.spawn_actor(walker,trans)
#         world.wait_for_tick()


#         controller = world.spawn_actor(controller_bp,carla.Transform(),actor)
#         world.wait_for_tick()

#         controller.start()
#         controller.go_to_location(world.get_random_location_from_navigation())
#         actors.append(actor)
#         actors.append(controller)


#     print(actors)
#     while(1):
#         time.sleep(0.1)
        
# finally:
#     print(actors)      
# camera_actor = world.get_spectator()
# # Define a localização da passadeira
# transform = Transform(Location(x=0, y=0, z=0.05))

# # Encontra o blueprint para uma passadeira
# blueprint_library = world.get_blueprint_library()
# blueprints = [bp for bp in world.get_blueprint_library().filter('static.prop*')]
# for blueprint in blueprints:
#    print(blueprint.id)
#    for attr in blueprint:
#        print('  - {}'.format(attr))
# crosswalk_bp = blueprint_library.find('static.prop.fountain')

# # Cria a passadeira
# crosswalk = world.spawn_actor(crosswalk_bp, transform)


# blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
# # Modify the attributes of the blueprint to set image resolution and field of view.
# blueprint.set_attribute('image_size_x', '1920')
# blueprint.set_attribute('image_size_y', '1080')
# blueprint.set_attribute('fov', '110')
# # Set the time in seconds between sensor captures
# blueprint.set_attribute('sensor_tick', '1.0')

# transform = carla.Transform(carla.Location(x=0.8, z=1.7))
# sensor = world.spawn_actor(blueprint, transform, attach_to=crosswalk)
# # Define um tempo de vida para a passadeira

# object_transform = crosswalk.get_transform()


# camera_transform = camera_actor.get_transform()
# camera_transform.location = object_transform.location - 100 * object_transform.get_forward_vector()
# camera_transform.rotation = object_transform.rotation
# camera_actor.set_transform(camera_transform)