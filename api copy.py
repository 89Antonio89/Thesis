import matplotlib.pyplot as plt
import matplotlib
from asyncio import wait
import math
import queue
import sys
from flask import Flask, jsonify, request
from flask import Flask
import traci
import threading
import random
import carla
import logging
import json
import os
import io
import argparse
import json
import numpy as np
from sklearn.cluster import DBSCAN
import logging
import requests
import random
import re
import shutil
import tempfile
import time
import sumolib  # pylint: disable=wrong-import-position
import requests
import lxml.etree as ET  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position
from run_synchronization import SimulationSynchronization  # pylint: disable=wrong-import-position
from util.netconvert_carla import netconvert_carla
import weakref
import cv2
import numpy as np
import os
from functools import partial
import xml.etree.ElementTree as ET
import json
import traci.constants as tc
import struct
app = Flask(__name__)

# Create the lock
lock = threading.Lock()
condition = threading.Condition(lock)
steps = 10000
simulation_running = True
StatusSimulation = True
step_length = 0.1
port = 4000
fiwareURL = "http://192.168.1.191:8081"
routesFile = "configs/RoadConditions/Town04.rou.xml"
sensor_list = []
synchronization = None
sumo_net = None
sumo_simulation = None
carla_simulation = None
file_name = 'Town05'
sensor_list = []
cameras = []
world = None
netFile = "configs/Town05/Town05.net.xml"
routes = "configs/Town05/Town05.rou.xml"
configFile = "configs/Town05/Town05.sumocfg"
processing_thread = None
simulation_time = 300
matplotlib.use('Agg')
safe_distance = 100
collision_threshold = 20
safe_speed = 10
deceleration_duration = 1
radar_object = []
vehicle_radar_data = {}
sensor_network = {}
collision_threshold_distance = 30.0  # in meters
collisions_detected = 0

last_changed = {}
MAX_TIME_INTERVAL = 15  # 1 minute in seconds


fig, ax = plt.subplots()
heatmap = ax.imshow(np.zeros((100, 100)), cmap='hot', interpolation='nearest')


def camera_callback(image):
    image_data = np.array(image.raw_data)
    # image_data = image_data.reshape((image.height, image.width, 4))

    # # Convert BGRA to BGR
    # bgr_image = image_data[:, :, :3]

    # # Save the image to a file
    # cv2.imshow("", bgr_image)
    # cv2.waitKey(1)
    print(image_data)


def write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, additional_traci_clients=0):
    """
    Writes sumo configuration xml file.
    """
    root = ET.Element('configuration')

    input_tag = ET.SubElement(root, 'input')
    ET.SubElement(input_tag, 'net-file', {'value': net_file})
    ET.SubElement(input_tag, 'route-files', {'value': vtypes_file})

    gui_tag = ET.SubElement(root, 'gui_only')
    ET.SubElement(gui_tag, 'gui-settings-file', {'value': viewsettings_file})

    ET.SubElement(root, 'num-clients',
                  {'value': str(additional_traci_clients+1)})

    tree = ET.ElementTree(root)
    tree.write(cfg_file,
               encoding='UTF-8', xml_declaration=True)


def vehicle_obstacle_detector_callback(vehicle, obstacle_data):
    # Process the obstacle data to detect obstacles

    # Assuming the obstacle_data provides information about detected obstacles
    print(obstacle_data)

    # Update the Traci vehicle parameters based on the detected obstacles
    # # For example, slow down the vehicle if obstacles are detected
    # if obstacles_detected:
    #     traci.vehicle.setSpeed(vehicle.id, 10)  # Adjust the speed as needed
    # Optionally, use other Traci functions to change lanes or modify routes based on obstacles


def modify_rou_file(tau, sigma, impatience):
    tree = ET.parse("configs/Town05/Town05.rou.xml")
    root = tree.getroot()

    for vType in root.findall('vType'):
        if tau:
            vType.set('tau', str(tau))
        if sigma:
            vType.set('sigma', str(sigma))
        if impatience:
            # Replace 'politeness' with the actual XML attribute if different
            vType.set('impatience', str(impatience))

    tree.write("configs/Town05/Town05.rou.xml")  # Overwriting the original


@app.route('/modify_parameters', methods=['POST'])
def modify_parameters():
    # Extract the parameters from the request body
    data = request.json
    tau = data.get('tau', None)
    sigma = data.get('sigma', None)
    # Assuming politeness is a parameter you have
    impatience = data.get('impatience', None)

    # Call the function to modify the .rou.xml file
    modify_rou_file(tau, sigma, impatience)
    return jsonify({"message": "Parameters updated successfully!"})


def save_image(image):
    if not os.path.exists('images'):
        os.makedirs('images')
    try:
        filename = 'images/%.6d.jpg' % image.frame
        image.save_to_disk(filename)
        print(f"Saved image {filename}")
    except Exception as e:
        print(f"Error: {e}")


def simulation_step():
    traci.simulationStep()


def run_simulation():
    print("debug")
    while simulation_running:
        print("aquiring")
        lock.acquire()
        print("aquired")
        traci.simulationStep()
        lock.release()
        print("released")


def euclidean_clustering(points, eps, min_samples):
    clustering = DBSCAN(eps=eps, min_samples=min_samples)
    labels = clustering.fit_predict(points)
    return labels


# )def lidar_callback(sensor_data):
#     lidar_point_cloud = sensor_data.get('point_cloud', [])

#     # Convert the lidar_point_cloud to a numpy array (x, y, z)
#     points = np.array([[p.x, p.y, p.z] for p in lidar_point_cloud])

#     # Perform Euclidean clustering to group points into objects
#     eps = 0.5  # Adjust this value to control the clustering density
#     # Adjust this value to control the minimum number of points required for a cluster
#     min_samples = 5
#     object_labels = euclidean_clustering(points, eps, min_samples)

#     # Retrieve unique object IDs from the clustering results
#     unique_object_ids = np.unique(object_labels)

#     # Print the number of detected objects
#     print(f"Number of detected objects: {len(unique_object_ids)}")

#     # Loop through each detected object and print its number of points
#     for obj_id in unique_object_ids:
#         num_points_in_object = np.sum(object_labels == obj_id)
#         print(f"Object {obj_id}: {num_points_in_object} points"


def create_radar_callback(sumo_vehicle_id):
    def radar_callback(sensor_data):
        # Get relevant radar data
        radar_depth = ""
        for detect in sensor_data:
            radar_depth = detect.depth

        # Define a threshold for safe following distance
        safe_following_distance = 10.0  # Adjust this value as needed
        if radar_depth < safe_following_distance:
            # Calculate reduced speed based on the radar data
            reduced_speed = calculate_reduced_speed(
                radar_depth, safe_following_distance, sumo_vehicle_id)

            # Set the desired speed using SUMO API
            traci.vehicle.slowDown(sumo_vehicle_id, reduced_speed, 2)
            # print(f"reduced the {sumo_vehicle_id} velocity to {reduced_speed}")

    return radar_callback

# Define a function to calculate reduced speed based on distance


def calculate_reduced_speed(current_distance, safe_distance, sumo_vehicle_id):
    # Calculate a reduced speed based on the ratio of current distance to safe distance
    reduction_factor = current_distance / safe_distance
    min_speed = 5.0  # Minimum speed to maintain
    reduced_speed = min_speed + \
        (reduction_factor * (traci.vehicle.getMaxSpeed(sumo_vehicle_id) - min_speed))

    return reduced_speed


def attach_camera_to_traffic_light(traffic_light):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_pitch = -15.0  # Adjust the pitch angle
    # Use traffic light's yaw as camera's yaw
    camera_yaw = traffic_light.get_transform().rotation.yaw
    camera_roll = 0.0  # No roll angle
    # Create camera rotation
    camera_rotation = carla.Rotation(
        pitch=camera_pitch, yaw=camera_yaw, roll=camera_roll)

    # Calculate camera transform based on traffic light's position
    camera_location = traffic_light.get_location() + carla.Location(x=-5.5,
                                                                    z=2.8)  # Adjust x and z offsets
    camera_transform = carla.Transform(camera_location, camera_rotation)
    camera_bp.set_attribute('image_size_x', '640')
    camera_bp.set_attribute('image_size_y', '480')
    camera_bp.set_attribute('sensor_tick', '10.0')
    camera_bp.set_attribute('fov', '90')
    camera = world.spawn_actor(
        camera_bp, camera_transform, attach_to=traffic_light)
    camera.listen(camera_callback)
    return camera


def process_camera_data(image):
    image_data = np.array(image.raw_data)
    image_data = image_data.reshape((image.height, image.width, 4))

    # Convert BGRA to BGR
    bgr_image = image_data[:, :, :3]

    # Save the image to a file
    cv2.imshow("", bgr_image)
    cv2.waitKey(1)


##########################################

def attach_collision_sensor(vehicle, callback):
    # Find the blueprint for the sensor.
    collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')

    # Spawn the sensor and attach to vehicle.
    collision_sensor = world.spawn_actor(
        collision_sensor_bp, carla.Transform(), attach_to=vehicle)

    # Subscribe to its output.
    collision_sensor.listen(lambda event: callback(event))

    return collision_sensor


def attach_lidar_sensor(vehicle, callback):
    # Find the blueprint for the sensor.
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Set the attributes.
    lidar_bp.set_attribute('channels', '32')
    lidar_bp.set_attribute('range', '100')
    lidar_bp.set_attribute('points_per_second', '100000')
    lidar_bp.set_attribute('rotation_frequency', '10')
    lidar_bp.set_attribute('sensor_tick', '10.0')
    # Spawn the sensor and attach to the vehicle.
    lidar_sensor = world.spawn_actor(lidar_bp, carla.Transform(
        carla.Location(x=1.5, z=2.0)), attach_to=vehicle)

    # Subscribe to its output.
    lidar_sensor.listen(lambda data: callback(data))

    return lidar_sensor


def attach_radar_sensor(vehicle, callback):
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    rad_bp.set_attribute('horizontal_fov', str(35))
    rad_bp.set_attribute('vertical_fov', str(20))
    rad_bp.set_attribute('range', str(20))
    rad_bp.set_attribute('sensor_tick', '1.0')
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(pitch=5)
    rad_transform = carla.Transform(rad_location, rad_rotation)
    rad_ego = world.spawn_actor(
        rad_bp, rad_transform, attach_to=vehicle)
    rad_ego.listen(lambda data: callback(data))

    return rad_ego


def attach_obstacle_detector(vehicle, callback):
    world = vehicle.get_world()
    bp = world.get_blueprint_library().find('sensor.other.obstacle')
    transform = carla.Transform(carla.Location(x=2.5, z=0.7))
    sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
    sensor.listen(callback)
    return sensor

# def rad_callback(radar_data, sumo_id, actor_id):
#     # velocity_range = 7.5  # m/s
#     # current_rot = radar_data.transform.rotation
#     # for detect in radar_data:
#     #     azi = math.degrees(detect.azimuth)
#     #     alt = math.degrees(detect.altitude)
#     #     # The 0.25 adjusts a bit the distance so the dots can
#     #     # be properly seen
#     #     fw_vec = carla.Vector3D(x=detect.depth - 0.25)
#     #     carla.Transform(
#     #         carla.Location(),
#     #         carla.Rotation(
#     #             pitch=current_rot.pitch + alt,
#     #             yaw=current_rot.yaw + azi,
#     #             roll=current_rot.roll)).transform(fw_vec)

#     #     def clamp(min_v, max_v, value):
#     #         return max(min_v, min(value, max_v))

#     #     norm_velocity = detect.velocity / \
#     #         velocity_range  # range [-1, 1]
#     #     r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
#     #     g = int(
#     #         clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
#     #     b = int(
#     #         abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)

#     #     world.debug.draw_point(
#     #         radar_data.transform.location + fw_vec,
#     #         size=.075,
#     #         life_time=0.4,
#     #         persistent_lines=False,
#     #         color=carla.Color(r, g, b))
#     # print(sumo_id)
#     if sumo_id not in sensor_network:
#         sensor_network[sumo_id] = {}
#     sensor_network[sumo_id]['radar'] = radar_data


def lidar_callback(lidar_data, sumo_id, actor_id):
    global sensor_network
    if sumo_id not in sensor_network:
        sensor_network[sumo_id] = {}
    sensor_network[sumo_id]['lidar_data'] = lidar_data


def radar_callback(radar_data, sumo_id, actor_id):
    global sensor_network
    if sumo_id not in sensor_network:
        sensor_network[sumo_id] = {}
    sensor_network[sumo_id]['radar_data'] = radar_data


def obstacle_detector_callback(data, sumo_id, carla_id):
    global sensor_network
    if sumo_id not in sensor_network:
        sensor_network[sumo_id] = {}
    sensor_network[sumo_id]['obstacle_data'] = data


def collision_callback(event):
    actor_id = event.actor.id
    collided_with = event.other_actor.id
    impulse = event.normal_impulse
    intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
    collisions_detected += 1
    print(
        f"Collision detected: Actor {actor_id} collided with Actor {collided_with} with intensity {intensity}.")


def potential_collision(detected_objects):
    for obj in detected_objects:
        if obj['distance'] < collision_threshold:
            return True
    return False


def check_collision(radar_data):
    collision_threshold_distance = 5.0  # in meters
    collision_likely = False

    for obj in radar_data:
        # assuming velocity includes object distance for now
        distance = obj.velocity  # Change this line to use dot notation
        azimuth = obj.azimuth

        # Assuming 0 azimuth means the object is directly in front of us,
        # we can put a small window around this to look for potential collisions
        if -10 <= azimuth <= 10:
            if distance < collision_threshold_distance:
                collision_likely = True
                break

    return collision_likely


def adjust_trajectory(sumo_id, original_speed):
    current_speed = traci.vehicle.getSpeed(sumo_id)
    current_lane = traci.vehicle.getLaneIndex(sumo_id)
    num_lanes = traci.edge.getLaneNumber(traci.vehicle.getRoadID(sumo_id))

    # Slow down more intelligently
    new_speed = max(original_speed * 0.7, current_speed - 2)
    traci.vehicle.slowDown(sumo_id, new_speed, 1.0)
    # Lane change logic
    if num_lanes > 1:
        target_lane = (current_lane + 1) % num_lanes
        # Check if the target lane is free or not (this is a simplified check)
        vehicles_in_target_lane = traci.lane.getLastStepVehicleIDs(
            f"{traci.vehicle.getRoadID(sumo_id)}_{target_lane}")
        if not vehicles_in_target_lane:
            traci.vehicle.changeLane(sumo_id, target_lane, 10.0)


def closest_object_ahead(lidar_data):
    min_distance_ahead = float('inf')
    raw_data = lidar_data.raw_data

    # Unpack raw_data to get the point cloud as list of floats
    point_cloud = struct.unpack(f"{int(len(raw_data) / 4)}f", raw_data)

    # Iterate over point cloud four floats at a time (x, y, z, i)
    for i in range(0, len(point_cloud), 4):
        x, y, z, intensity = point_cloud[i:i + 4]

        # Considering only points that are in front of the vehicle
        if x > 0:
            distance = (x ** 2 + y ** 2 + z ** 2) ** 0.5
            if distance < min_distance_ahead:
                min_distance_ahead = distance

    return min_distance_ahead


def traffic_jam_assist(sumo_id):
    radar_data = sensor_network[sumo_id]['radar_data']
    lidar_data = sensor_network[sumo_id]['lidar_data']
    # obstacle_data = sensor_network[sumo_id]['obstacle_data']

    # min_safe_distance = 5.0  # in meters
    # current_speed = traci.vehicle.getSpeed(sumo_id)

    # # Get distance from the obstacle detector.
    # obstacle_distance = obstacle_data.distance
    # print(obstacle_distance)
    # # Process radar and lidar data to get information about surrounding vehicles
    # # ...
    # if obstacle_distance < min_safe_distance:
    #     # Slow down
    #     new_speed = max(0, current_speed - 2)
    #     # Slow down over 1 second
    #     traci.vehicle.slowDown(sumo_id, new_speed, 1.0)
    #     # Optionally, check if it's safe to change lanes and do so
    #     # ...

    # else:
    #     # Resume normal speed
    #     traci.vehicle.setSpeed(sumo_id, current_speed + 2)


def utility_max_min(lane):
    """
    Utility function for Max-Min fairness. The function prioritizes lanes that are fully congested.
    We calculate a utility based on the percentage of lane occupancy.
    """
    total_cars = traci.lane.getLastStepVehicleNumber(lane)
    # Assuming car length + gap is roughly 1 unit
    lane_capacity = traci.lane.getLength(lane)
    lane_occupancy_percentage = total_cars / lane_capacity

    return lane_occupancy_percentage


def manage_traffic_lights():
    # Constants
    # Change traffic light if it's been 12 seconds since the last change
    MAX_TIME_INTERVAL = 15

    current_time = traci.simulation.getTime()
    traffic_lights = traci.trafficlight.getIDList()

    global last_changed

    for tl in traffic_lights:
        max_utility = -float('inf')
        most_prioritized_lane = None

        controlled_lanes = traci.trafficlight.getControlledLanes(tl)

        for lane in controlled_lanes:
            lane_utility = utility_max_min(lane)
            if lane_utility > max_utility:
                max_utility = lane_utility
                most_prioritized_lane = lane

        # Check conditions to change the traffic light:
        can_change = (tl not in last_changed) or (
            current_time - last_changed[tl] >= MAX_TIME_INTERVAL) or (max_utility == 0)
        if not can_change:
            continue

        # Use the utility function's result to set the traffic light phase
        logic = traci.trafficlight.getAllProgramLogics(tl)[0]
        for i, phase in enumerate(logic.phases):
            if most_prioritized_lane and phase.state[controlled_lanes.index(most_prioritized_lane)] == 'g':
                traci.trafficlight.setPhase(tl, i)
                last_changed[tl] = current_time  # Update the last changed time
                break


def lidar_to_2d_matrix(lidar_data, grid_size):
    """
    Convert LiDAR data to 2D matrix representation.

    Parameters:
        - lidar_data: The filtered LiDAR point cloud data.
        - grid_size: The size of the grid for the 2D matrix (e.g., 1m x 1m cells).

    Returns:
        - A 2D matrix representation of the LiDAR data.
    """
    # Determine the bounding box of the data
    max_x = max([point.point.x for point in lidar_data])
    min_x = min([point.point.x for point in lidar_data])
    max_y = max([point.point.y for point in lidar_data])
    min_y = min([point.point.y for point in lidar_data])

    # Create an empty matrix based on the bounding box and grid size
    matrix = np.zeros((int((max_x - min_x) // grid_size) + 1,
                       int((max_y - min_y) // grid_size) + 1))

    for point in lidar_data:
        x_index = (point.point.x - min_x) // grid_size
        y_index = (point.point.y - min_y) // grid_size
        matrix[int(x_index), int(y_index)] += 1

    return matrix


def display_heatmap(matrix):
    plt.imshow(matrix, cmap='viridis', interpolation='bilinear')
    plt.colorbar()
    file_name = f"/test/heatmap_{np.random.randint(1000)}.png"
    plt.savefig(file_name, dpi=300)


def lidar_data_callbacks(data):
    # For now, just print some info from the data
    # adjust grid_size as required
    matrix = lidar_to_2d_matrix(data, grid_size=1)
    display_heatmap(matrix)


def attach_to_traffic_lights_lidar():
    all_traffic_lights = [
        actor for actor in world.get_actors() if 'traffic_light' in actor.type_id]
    print(len(all_traffic_lights))
    for traffic_light in all_traffic_lights:
        sensor = attach_lidar_sensor(traffic_light, lidar_data_callbacks)
        sensor_list.append(sensor)


@app.route('/startSimulation', methods=['POST'])
def start():
    start_time = time.time()
    try:
        # attach_to_traffic_lights_lidar()
        while simulation_running:
            current_time = time.time()
            if current_time - start_time > simulation_time:
                print(collisions_detected, "collisions detected")
                break
            if lock.locked():
                lock.release()
            lock.acquire()
            start = time.time()
            synchronization.tick()
            actor_list = world.get_actors()

            new_vehicles = traci.simulation.getDepartedIDList()
            for sumo_id in new_vehicles:
                if sumo_id in synchronization.sumo2carla_ids:
                    actor_id = synchronization.sumo2carla_ids[sumo_id]
                    carla_vehicle = carla_simulation.client.get_world().get_actor(actor_id)
                    # traci.vehicle.setMinGap(sumo_id, 1)

                    # radar_sensor = attach_radar_sensor(
                    #     carla_vehicle, lambda radar_data, sumo_id=sumo_id, actor_id=actor_id: radar_callback(radar_data, sumo_id, actor_id))
                    # sensor_list.append(radar_sensor)

                    # lidar_sensor = attach_lidar_sensor(
                    #     carla_vehicle, lambda lidar_data: lidar_callback(lidar_data, sumo_id, actor_id))
                    # sensor_list.append(lidar_sensor)

                    # obstacle_detector = attach_obstacle_detector(
                    #     carla_vehicle, lambda data: obstacle_detector_callback(data, sumo_id, actor_id))
                    # sensor_list.append(obstacle_detector)
                    collision_sensor = attach_collision_sensor(
                        carla_vehicle, collision_callback)
                    sensor_list.append(collision_sensor)
            # for sumo_id, data in sensor_network.items():
            #     radar_data = data.get('radar_data')
            #     lidar_data = data.get('lidar_data')
            #     obstacle_data = data.get('obstacle_data')

            #     if lidar_data is not None and radar_data is not None and obstacle_data is not None:
            #         traffic_jam_assist(sumo_id)
            manage_traffic_lights()
            end = time.time()
            elapsed = end - start
            if elapsed < step_length:
                time.sleep(step_length - elapsed)
            lock.release()
    except Exception as e:
        logging.exception("Unhandled Error: %s", str(e))
    return "done"


@app.route('/initSumo', methods=['GET'])
def init_sumo():
    print("acquired")
    traci.init(3001)
    print("here1")
    traci.setOrder(2)
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.start()
    print("here1")

    return 'Simulation Initialized'


@ app.route('/initSimulation', methods=['GET'])
def init_simulation():
    basedir = os.getcwd()

    # Read file content
    xodr_file_path = 'CampoAlegre.xodr'
    with io.open(xodr_file_path, 'r', encoding="utf-8") as f:
        xodr_content = f.read()

    vertex_distance = 2.0  # in meters
    max_road_length = 50.0  # in meters
    wall_height = 0.0      # in meters
    extra_width = 0.6      # in meters
    world = client.generate_opendrive_world(
        xodr_content, carla.OpendriveGenerationParameters(
            vertex_distance=vertex_distance,
            max_road_length=max_road_length,
            wall_height=wall_height,
            additional_width=extra_width,
            smooth_junctions=True,
            enable_mesh_visibility=True))
 # Temporal folder to save intermediate files.
    tmpdir = tempfile.mkdtemp()

    # ----------------
    # carla simulation
    # ----------------
    carla_simulation = CarlaSimulation(
        '127.0.0.1', 2000, step_length)

    world = carla_simulation.client.get_world()
    current_map = world.get_map()

    xodr_file = os.path.join(tmpdir, current_map.name + '.xodr')
    current_map.save_to_disk(xodr_file)

    # ---------------
    # sumo simulation
    # ---------------
    net_file = os.path.join(tmpdir, current_map.name + '.net.xml')
    netconvert_carla(xodr_file, net_file, guess_tls=True)

    basedir = os.path.dirname(os.path.realpath(__file__))
    cfg_file = os.path.join(tmpdir, current_map.name + '.sumocfg')
    vtypes_file = os.path.join(basedir, 'examples', 'carlavtypes.rou.xml')
    viewsettings_file = os.path.join(basedir, 'examples', 'viewsettings.xml')
    write_sumocfg_xml(cfg_file, net_file, vtypes_file,
                      viewsettings_file, 0)

    sumo_net = sumolib.net.readNet(net_file)

    sumo_simulation = SumoSimulation(cfg_file,
                                     step_length,
                                     host=None,
                                     port=None,
                                     sumo_gui=False,
                                     client_order=1)

    print("done sumo init")
    # ---------------
    # synchronization
    # ---------------
    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, 'sumo',
                                                False, True)

    try:
        # ----------
        # Blueprints
        # ----------
        with open('data/vtypes.json') as f:
            vtypes = json.load(f)['carla_blueprints']

        blueprints = vtypes.keys()

        filterv = re.compile('vehicle.*')
        blueprints = list(filter(filterv.search, blueprints))

        if True:
            blueprints = [
                x for x in blueprints if vtypes[x]['vClass'] not in ('motorcycle', 'bicycle')
            ]
            blueprints = [x for x in blueprints if not x.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.endswith('carlacola')]
            blueprints = [
                x for x in blueprints if not x.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.endswith('t2')]
            blueprints = [x for x in blueprints if not x.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.endswith('ambulance')]

        if not blueprints:
            raise RuntimeError(
                'No blueprints available due to user restrictions.')

        # # --------------
        # # Spawn vehicles
        # # --------------
        # # Spawns sumo NPC vehicles.
        # sumo_edges = sumo_net.getEdges()

        # for i in range(5):
        #     type_id = random.choice(blueprints)
        #     vclass = vtypes[type_id]['vClass']

        #     allowed_edges = [e for e in sumo_edges if e.allows(vclass)]
        #     if allowed_edges:
        #         edge = random.choice(allowed_edges)

        #         traci.route.add('route_{}'.format(i), [edge.getID()])
        #         traci.vehicle.add('sumo_{}'.format(
        #             i), 'route_{}'.format(i), typeID=type_id)
        #     else:
        #         logging.error(
        #             'Could not found a route for %s. No vehicle will be spawned in sumo',
        #             type_id)

        while True:
            lock.acquire()
            start = time.time()

            synchronization.tick()

            # # Updates vehicle routes
            # for vehicle_id in traci.vehicle.getIDList():
            #     route = traci.vehicle.getRoute(vehicle_id)
            #     index = traci.vehicle.getRouteIndex(vehicle_id)
            #     vclass = traci.vehicle.getVehicleClass(vehicle_id)

            #     if index == (len(route) - 1):
            #         current_edge = sumo_net.getEdge(route[index])
            #         available_edges = list(
            #             current_edge.getAllowedOutgoing(vclass).keys())
            #         if available_edges:
            #             next_edge = random.choice(available_edges)

            #             new_route = [current_edge.getID(), next_edge.getID()]
            #             traci.vehicle.setRoute(vehicle_id, new_route)

            end = time.time()
            elapsed = end - start
            if elapsed < step_length:
                time.sleep(step_length - elapsed)
            lock.release()
    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        synchronization.close()

        if os.path.exists(tmpdir):
            shutil.rmtree(tmpdir)


@app.route('/initSimulation', methods=['GET'])
def init():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        world = client.get_world()
    finally:
        return "Simulation Connected"


def camera_thread():
    while True:
        if sensor_list:
            world.tick()
        else:
            time.sleep(0.1)


@app.route('/AttachCameras', methods=['GET'])
def AttachCameras():
    lock.acquire()
    cam_bp = None
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x", str(1920))
    cam_bp.set_attribute("image_size_y", str(1080))
    cam_bp.set_attribute("fov", str(105))
    cam_bp.set_attribute("sensor_tick", '1.0')
    cam_location = carla.Location(2, 0, 1)
    cam_rotation = carla.Rotation(0, 0, 0)
    cam_transform = carla.Transform(cam_location, cam_rotation)
    vehicles = world.get_actors().filter('vehicle.*')
    for vehicle in vehicles:
        ego_cam = world.spawn_actor(
            cam_bp, cam_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        ego_cam.listen(lambda image: save_image(image))
        sensor_list.append(ego_cam)
    print(f"Attached {len(sensor_list)} cameras:")
    for sensor in sensor_list:
        print(sensor.id)

    if len(sensor_list) == 1:
        t = threading.Thread(target=camera_thread)
        t.start()
    # traffic_lights = []
    # for actor in world.get_actors():
    #     if actor.type_id == 'traffic.traffic_light':
    #         parent_actor = actor.get_parent()
    #         if isinstance(parent_actor, carla.libcarla.Road):
    #             traffic_lights.append(actor)
    # print(traffic_lights)
    # cameras = []
    # for traffic_light in traffic_lights:
    #     camera = attach_camera_to_traffic_light(traffic_light)
    #     cameras.append(camera)
    #     cam = camera.get_transform()
    #     time.sleep(5)
    #     spectator = world.get_spectator()
    #     spectator.set_transform(cam)
    # print(cameras)
    lock.release()
    print("Waiting for images...")
    return "appended cameras"


@app.route('/getActors', methods=['GET'])
def getActors():
    lock.acquire()
    actors_list = world.get_actors().filter('traffic*')
    lock.release()
    return jsonify(actors_list)


@app.route('/carsInfo', methods=["GET"])
def carsInfo():
    lock.acquire()
    vehicle_ids = traci.vehicle.getIDList()
    vehicle_info = {}
    for v_id in vehicle_ids:
        laneid = traci.vehicle.getLaneID(v_id)
        typeid = traci.vehicle.getTypeID(v_id)
        pos = traci.vehicle.getPosition(v_id)
        speed = traci.vehicle.getSpeed(v_id)
        vehicle_info[v_id] = {"position": pos,
                              "speed": speed, "laneID": laneid, "typeid": typeid}
    lock.release()
    return jsonify(vehicle_info)


@app.route('/endSimulation', methods=["GET"])
def endSimulation():
    lock.acquire()
    traci.close()
    time.sleep(4)
    lock.release()
    return "simulation ended"


@app.route('/insertCar', methods=["POST"])
async def insertCar():
    data = request.get_json()
    cars = ""
    for x in data:
        print(x)
        id = x['id']
        type = x['type']
        lock.acquire()
        edge_list = traci.edge.getIDList()
        with open('data/vtypes.json') as f:
            vtypes = json.load(f)['carla_blueprints']
        # print(vtypes)
        type_id = random.choice(blueprints)
        vclass = vtypes[type]['vClass']
        allowed_edges = [e for e in sumo_edges if e.allows(vclass)]
        if allowed_edges:
            edge = random.choice(allowed_edges)
            traci.route.add('route_{}'.format(id), [edge.getID()])
            traci.vehicle.add('sumo_{}'.format(
                id), 'route_{}'.format(id), typeID=type)
            cars += ' sumo_{}'.format(id)
            lock.release()
        else:
            lock.release()

    return "inserted cars" + cars


@app.route('/getRoutes', methods=["GET"])
def routes():
    lock.acquire()
    routes = traci.route.getIDList()
    lock.release()
    return jsonify(routes)


@app.route('/getvTypes', methods=["GET"])
def vTypes():
    return jsonify(blueprints)


@app.route('/set-weather', methods=["POST"])
def setWeather():
    data = request.get_json()
    try:
        world = client.get_world()
        weather = world.get_weather()
        weather.cloudiness = data['cloudcover']
        weather.wetness = data['humidity']
        weather.precipitation = data['precipitation']
        weather.precipitation_deposits = data['precipitation_deposits']
        weather.wind_intensity = data['wind_intensity']
        weather.fog_density = data['fog_density']
        weather.sun_azimuth_angle = data['sun_azimuth_angle']
        weather.sun_altitude_angle = data['sun_altitude_angle']
        weather.dust_storm = data['dust_storm']
        world.set_weather(weather)
    except e:
        print(e, "Could not set weather parameters")
    return "weather set"


@app.route('/saveNetwork', methods=["GET"])
def saveNetwork():
    try:
        lock.acquire()
        filename = request.args.get('filename')
        traci.simulation.saveState(filename)
        lock.release()
    finally:
        print("saved network")
    return "saved network"


@ app.route('/loadNetwork', methods=["GET"])
def loadNetwork():
    try:
        filename = request.args.get('filename')
        lock.acquire()
        traci.simulation.loadState(filename)
        lock.release()
    finally:
        print("Network loaded")
    return "Network loaded"


@ app.route('/getEdges', methods=["GET"])
def getEdges():
    try:
        edges = traci.edge.getIDList()

    except:
        return "Could not retrieve edges"
    finally:
        return edges


def save_rgb_data(output_folder, frame_number):
    def callback(sensor_data):
        # Get the image data from the sensor
        image_data = sensor_data.raw_data
        os.makedirs(output_folder, exist_ok=True)
        # Save the RGB image to the specified folder
        filename = os.path.join(output_folder, f"{frame_number}.png")
        with open(filename, 'wb') as file:
            file.write(image_data)

    return callback


@ app.route('/testCameras', methods=["GET"])
def cam():
    try:
        return "asd"
    except:
        print("error")
        return "error"
    finally:
        for sensor in sensor_list:
            sensor.stop()
            sensor.destroy()
        print("wheater set ")
        return "set cameras"


@ app.route('/endSumoConnection', methods=["GET"])
def close():
    global simulation_running
    try:

        synchronization.close()
        sumo_simulation.close()
        simulation_running = False
        for sensor in sensor_list:
            sensor.stop()
            sensor.destroy()
        for camera in cameras:
            camera.destroy()
        for actors in world.get_actors():
            actors.destroy()
        lock.release()
    except:
        return "Unhandle error"
    finally:
        return "ended simulation"


def reload_simulation():
    global synchronization
    global sumo_net
    global sumo_simulation
    try:
        sumo_simulation = SumoSimulation(configFile,
                                         step_length,
                                         host=None,
                                         port=None,
                                         sumo_gui=False,
                                         client_order=1)
        # ---------------
        # synchronization
        # ---------------
        synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, 'sumo',
                                                    False, True)
    finally:
        start()
        print("Reloaded Simulation")


@app.route('/reloadSimulation', methods=["GET"])
def reload():
    simulation_running = True
    # Create a new thread for executing the reload_simulation function
    thread = threading.Thread(target=reload_simulation)

    # Start the thread
    thread.start()

    return "Reloading simulation..."


def process_image(image):
    # Do something with the image
    image.save_to_disk('output_camera.png')


@ app.route('/getEntities', methods=["GET"])
def getEntities():
    try:
        tree = ET.parse(routesFile)
        root = tree.getroot()

        xml_content = '''<?xml version="1.0" encoding="UTF-8"?>
        <!-- generated on 2023-05-27 16:35:51 by Eclipse SUMO netedit Version 1.17.0
    -->
    <routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
        <!-- VTypes -->
        <vType id="vehicle.audi.a2" length="3.72" vClass="passenger" width="1.79" height="1.54"/>
        <vType id="vehicle.audi.etron" length="4.89" vClass="evehicle" width="2.09" height="1.65"/>
        <vType id="vehicle.audi.tt" length="4.15" vClass="passenger" width="2.00" height="1.38"/>
        <vType id="vehicle.bh.crossbike" length="1.51" vClass="bicycle" width="0.87" height="1.53"/>
        <vType id="vehicle.bmw.grandtourer" length="4.64" vClass="passenger" width="2.26" height="1.68"/>
        <vType id="vehicle.carlamotors.carlacola" length="5.20" vClass="truck" guiShape="truck" width="2.62" height="2.48"/>
        <vType id="vehicle.chevrolet.impala" length="5.37" vClass="passenger" width="2.05" height="1.42"/>
        <vType id="vehicle.citroen.c3" length="3.98" vClass="passenger" width="1.86" height="1.63"/>
        <vType id="vehicle.diamondback.century" length="1.66" vClass="bicycle" width="0.42" height="1.49"/>
        <vType id="vehicle.dodge.charger_police" length="5.01" vClass="authority" guiShape="police" width="2.05" height="1.58"/>
        <vType id="vehicle.ford.mustang" length="4.90" vClass="passenger" width="2.06" height="1.48"/>
        <vType id="vehicle.gazelle.omafiets" length="1.84" vClass="bicycle" width="0.47" height="1.59"/>
        <vType id="vehicle.harley-davidson.low_rider" length="2.35" vClass="motorcycle" width="0.77" height="1.46"/>
        <vType id="vehicle.jeep.wrangler_rubicon" length="3.87" vClass="passenger" width="1.91" height="1.88"/>
        <vType id="vehicle.kawasaki.ninja" length="2.04" vClass="motorcycle" width="0.80" height="1.37"/>
        <vType id="vehicle.lincoln.mkz_2017" length="4.90" vClass="passenger" width="2.06" height="1.48"/>
        <vType id="vehicle.mercedes.coupe" length="5.04" vClass="passenger" width="2.16" height="1.64"/>
        <vType id="vehicle.micro.microlino" length="2.20" vClass="evehicle" width="1.49" height="1.35"/>
        <vType id="vehicle.mini.cooper_s" length="3.80" vClass="passenger" width="1.97" height="1.48"/>
        <vType id="vehicle.nissan.micra" length="3.67" vClass="passenger" width="1.86" height="1.52"/>
        <vType id="vehicle.nissan.patrol" length="4.52" vClass="passenger" width="1.92" height="1.89"/>
        <vType id="vehicle.seat.leon" length="4.21" vClass="passenger" width="1.82" height="1.47"/>
        <vType id="vehicle.tesla.cybertruck" length="6.36" vClass="evehicle" width="2.39" height="2.14"/>
        <vType id="vehicle.tesla.model3" length="4.81" vClass="evehicle" width="2.17" height="1.52"/>
        <vType id="vehicle.toyota.prius" length="4.54" vClass="evehicle" width="2.00" height="1.54"/>
        <vType id="vehicle.volkswagen.t2" length="4.47" vClass="passenger" guiShape="passenger/van" width="1.81" height="2.05"/>
        <vType id="vehicle.yamaha.yzf" length="2.19" vClass="motorcycle" width="0.77" height="1.63"/>

        '''

        url = fiwareURL + '/entity/list?noDummies=true'
        response = requests.get(url)

        if response.status_code == 200:
            data = response.json()
            for item in data["results"]:
                if item.get("type") == "Trip":
                    # Create a new "trip" element
                    trip_element = ET.Element("trip")
                    # Set the attributes of the "trip" element
                    trip_element.set("id", item.get(
                        "tripId", {}).get("value", ""))
                    trip_element.set("type", item.get(
                        "blueprintType", {}).get("value", ""))
                    trip_element.set("depart", item.get(
                        "depart", {}).get("value", ""))
                    trip_element.set("from", item.get(
                        "from", {}).get("value", ""))
                    trip_element.set("to", item.get(
                        "to", {}).get("value", ""))

                    # Append the "trip" element to the XML content
                    xml_content += "\n" + ET.tostring(trip_element,
                                                      encoding="unicode")

            xml_content += "</routes>"  # Close the root element

        else:
            print('Request failed with status code:', response.status_code)
            return

        file_path = routesFile

        if os.path.isfile(file_path):
            with open(file_path, 'w') as file:
                file.write(xml_content)
            print("XML file content replaced successfully.")
        else:
            print("File does not exist.")
    except ET.ParseError as e:
        print("Error occurred while parsing the XML file:", e)
    except requests.RequestException as e:
        print("Request failed:", e)
    except Exception as e:
        print("An error occurred:", e)
    finally:
        return "done"


def saveEntities():
    tree = ET.parse(routesFile)
    root = tree.getroot()

    # Define a list to store the entities
    entities = []

    # Iterate over the vType elements and extract the required attributes
    # for vType_elem in root.iter('vType'):
    #     vType = {
    #         'id': vType_elem.get('id'),
    #         'vClass': vType_elem.get('vClass'),
    #         'length': vType_elem.get('length'),
    #         'width': vType_elem.get('width'),
    #         'height': vType_elem.get('height')
    #     }
    #     entities.append(vType)

    # Iterate over the trip elements and extract the required attributes
    i = 1
    for trip_elem in root.iter('trip'):
        trip = {
            'id': f'trip:Trip:{i}',
            'type': 'Trip',
            'tripId': trip_elem.get('id'),
            'blueprintType': {
                'type': 'string',
                'value': trip_elem.get('type'),
                'metadata': {}
            },
            'depart': {
                'type': 'string',
                'value': trip_elem.get('depart'),
                'metadata': {}
            },
            'from': {
                'type': 'string',
                'value': trip_elem.get('from'),
                'metadata': {}
            },
            'to': {
                'type': 'string',
                'value': trip_elem.get('to'),
                'metadata': {}
            }
        }
        i += 1
        entities.append(trip)

    # Convert the entities list to JSON
    json_data = json.dumps(entities, indent=4)

    # Save the JSON data to a file
    with open('entities.json', 'w') as json_file:
        json_file.write(json_data)
    return


def set_road_friction(road_ids, friction_value):
    for road_id in road_ids:
        traci.edge.setFriction(road_id, str(friction_value))


@app.route('/testFriction', methods=["GET"])
def testFriction():
    all_road_ids = traci.edge.getIDList()

    set_road_friction(all_road_ids, 0.2)
    return "good"


@app.route('/changePhysics', methods=["GET"])
def changePhysics():
    try:
        weather = world.get_weather()

        # Set the desired weather conditions
        weather.cloudiness = 80  # Adjust the cloudiness value (0-100)
        weather.precipitation = 80  # Adjust the precipitation value (0-100)
        # Adjust the precipitation deposits value (0-100)
        weather.precipitation_deposits = 80
        weather.wind_intensity = 60  # Adjust the wind intensity value (0-100)
        # Adjust the sun azimuth angle value (0-360)
        weather.sun_azimuth_angle = 45
        # Adjust the sun altitude angle value (0-90)
        weather.sun_altitude_angle = 30

        # Apply the modified weather conditions to the world
        world.set_weather(weather)
        # setAllTrafficLightsToGreen()
        surface_conditions = {
            'normal': {
                'tire_friction': 1.0,
                'steer_sensitivity': 1.0
            },
            'wet': {
                'tire_friction': 0.8,
                'steer_sensitivity': 0.8
            },
            'icy': {
                'tire_friction': 0.1,
                'steer_sensitivity': 0.1
            }
        }
        current_surface = 'icy'

        dynamics_params = surface_conditions[current_surface]
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            for param, value in dynamics_params.items():
                traci.vehicle.setParameter(
                    vehicle_id, f'vehicle.{param}', str(value))

    except Exception as e:
        logging.exception("Unhandled Error: %s", str(e))
        return str(e)
    finally:
        return "New physics have been applied to SUMO vehicles."


def setAllTrafficLightsToGreen():
    try:
        # Retrieve the list of traffic light IDs in the simulation
        traffic_light_ids = traci.trafficlight.getIDList()

        # Set all phases to green for each traffic light
        for tl_id in traffic_light_ids:
            program_id = traci.trafficlight.getProgram(tl_id)
            traci.trafficlight.setProgram(576, program_id)

    except Exception as e:
        logging.exception("Unhandled Error: %s", str(e))
        return str(e)
    finally:
        return "All traffic lights have been set to green."


environmental_factors = {
    'normal': {
        'tire_friction': 1.0,
        'steer_sensitivity': 1.0
    },
    'wet': {
        'tire_friction': 0.8,
        'steer_sensitivity': 0.8
    },
    'icy': {
        'tire_friction': 0.1,
        'steer_sensitivity': 0.1
    }
}


@app.route('/change_environment', methods=['POST'])
def change_environment():
    try:
        # Map environmental data to SUMO vehicle parameters
        tire_friction = 0.1
        steer_sensitivity = 0.1
        vehicle_list = traci.vehicle.getIDList()

        # Update SUMO vehicle parameters
        for vehicle_id in vehicle_list:
            traci.vehicle.setSpeedFactor(vehicle_id, 1.5)
            traci.vehicle.setSpeedDeviation(vehicle_id, 1.5)
            traci.vehicle.setAccel(vehicle_id, 1.5)
            traci.vehicle.setDecel(vehicle_id, 1.5)
            traci.vehicle.setTireFrictionCoefficient(
                vehicle_id, steer_sensitivity)
            traci.vehicle.setSteerSensitivity(vehicle_id, steer_sensitivity)
    except Exception as e:
        logging.exception("Unhandled Error: %s", str(e))
        return str(e)
    finally:
        return "All traffic lights have been set to green."


def set_carla_weather(weather_type):
    if weather_type == 'rainy':
        weather = carla.WeatherParameters(
            precipitation=80.0,
            precipitation_deposits=30.0,
            fog_density=0.0,
            fog_distance=0.0,
            wetness=100.0,
            wind_intensity=50.0,
            sun_azimuth_angle=45.0,
            sun_altitude_angle=30.0
        )
    elif weather_type == 'icy':
        weather = carla.WeatherParameters(
            precipitation=20.0,
            precipitation_deposits=80.0,
            fog_density=0.0,
            fog_distance=0.0,
            wetness=100.0,
            wind_intensity=20.0,
            sun_azimuth_angle=90.0,
            sun_altitude_angle=15.0
        )
    else:  # Normal atmosphere
        weather = carla.WeatherParameters(
            precipitation=0.0,
            precipitation_deposits=0.0,
            fog_density=0.0,
            fog_distance=0.0,
            wetness=0.0,
            wind_intensity=0.0,
            sun_azimuth_angle=180.0,
            sun_altitude_angle=45.0
        )
    # Set the weather conditions in CARLA
    world.set_weather(weather)


def adjust_vehicle_parameters(vehicle_id, weather_type):
    if weather_type == 'rainy':
        acceleration_factor = 0.8
        deceleration_factor = 0.9
        max_speed_factor = 0.9
    elif weather_type == 'icy':
        acceleration_factor = 0.5
        deceleration_factor = 0.7
        max_speed_factor = 0.7
    else:  # Normal atmosphere
        acceleration_factor = 1.0
        deceleration_factor = 1.0
        max_speed_factor = 1.0

        # Adjust acceleration
    acceleration = traci.vehicle.getAcceleration(
        vehicle_id) * acceleration_factor
    traci.vehicle.setAcceleration(vehicle_id, acceleration, duration=1)

    # Adjust deceleration
    deceleration = traci.vehicle.getDecel(vehicle_id) * deceleration_factor
    traci.vehicle.setDecel(vehicle_id, deceleration)

    # Adjust maximum speed
    max_speed = traci.vehicle.getMaxSpeed(vehicle_id) * max_speed_factor
    traci.vehicle.setMaxSpeed(vehicle_id, max_speed)


def synchronize_carla_sumo_weather(vehicle_id, weather_type):
    adjust_vehicle_parameters(vehicle_id, weather_type)


@app.route('/setWeather', methods=["POST"])
def set_weather():
    try:
        weather_type = request.form.get('weather_type')
        synchronize_carla_sumo_weather(weather_type)
        return f"Weather set to: {weather_type}"
    except Exception as e:
        return f"Error setting weather: {str(e)}"


if __name__ == "__main__":
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    try:
        client = carla.Client('127.0.0.1', port)
        client.set_timeout(60.0)
        world = client.get_world()

        basedir = os.getcwd()
        map_name = world.get_map().name
        split_path = map_name.split('/')

        map_name = split_path[-1]
        # if map_name != file_name:
        world = client.load_world(file_name)
        tmpdir = tempfile.mkdtemp()

        # ----------------
        # carla simulation
        # ----------------
        carla_simulation = CarlaSimulation(
            '127.0.0.1', port, step_length)

        world = carla_simulation.client.get_world()
        current_map = world.get_map()

        xodr_file = os.path.join(tmpdir, file_name + '.xodr')
        current_map.save_to_disk(xodr_file)

        # ---------------
        # sumo simulation
        # ---------------
        # net_file = file_name + '.net.xml'
        # netconvert_carla (xodr_file, net_file, guess_tls=True)

        # basedir = os.path.dirname(os.path.realpath(__file__))
        # cfg_file = file_name + '.sumocfg'
        # vtypes_file = (routesFile)
        # viewsettings_file = ('examples/' + 'viewsettings.xml')
        # write_sumocfg_xml(cfg_file, net_file, routesFile,
        #                   viewsettings_file, 0)

        sumo_net = sumolib.net.readNet(netFile)

        sumo_simulation = SumoSimulation(configFile,
                                         step_length,
                                         host=None,
                                         port=None,
                                         sumo_gui=False,
                                         client_order=1)

        print("done sumo init")
        # ---------------
        # synchronization
        # ---------------
        synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, 'sumo',
                                                    False, True)

        with open('data/vtypes.json') as f:
            vtypes = json.load(f)['carla_blueprints']
        blueprints = vtypes.keys()
        filterv = re.compile('vehicle.*')
        blueprints = list(filter(filterv.search, blueprints))

        if True:
            blueprints = [
                x for x in blueprints if vtypes[x]['vClass'] not in ('motorcycle', 'bicycle')
            ]
            blueprints = [x for x in blueprints if not x.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.endswith('carlacola')]
            blueprints = [
                x for x in blueprints if not x.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.endswith('t2')]
            blueprints = [x for x in blueprints if not x.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.endswith('ambulance')]

        sumo_edges = sumo_net.getEdges()
        map = world.get_map()
        # spawn_points = map.get_spawn_points()
        # center = carla.Location()
        # for spawn_point in spawn_points:
        #     center.x += spawn_point.location.x
        #     center.y += spawn_point.location.y
        #     center.z += spawn_point.location.z
        # center.x /= len(spawn_points)
        # center.y /= len(spawn_points)
        # center.z /= len(spawn_points)
        # spectator = world.get_spectator()
        # spectator.set_transform(carla.Transform(center, carla.Rotation()))
        app.run(host='0.0.0.0')
        # start() is not called

    # saveEntities()
    except Exception as e:
        logging.exception("Unhandled Error: %s", str(e))
    finally:
        for sensor in sensor_list:
            sensor.stop()
            sensor.destroy()
        for camera in cameras:
            camera.destroy()
        for actors in world.get_actors():
            actors.destroy()
        sensor_list = []
        cameras = []
        synchronization.close()
        sumo_simulation.close()
        print("end of connection")
