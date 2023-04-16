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
import logging
import requests
import random
import re
import shutil
import tempfile
import time
import sumolib  # pylint: disable=wrong-import-position
import traci  # pylint: disable=wrong-import-position
import requests
import lxml.etree as ET  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position
from run_synchronization import SimulationSynchronization  # pylint: disable=wrong-import-position
from util.netconvert_carla import netconvert_carla
app = Flask(__name__)

# Create the lock
lock = threading.Lock()
condition = threading.Condition(lock)
steps = 10000
simulation_running = True


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
    tree.write(cfg_file, pretty_print=True,
               encoding='UTF-8', xml_declaration=True)


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


@app.route('/initSumo', methods=['GET'])
def init_sumo():
    print("acquiredd")
    traci.init(3001)
    print("here1")
    traci.setOrder(2)
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.start()
    print("here1")

    return 'Simulation Initialized'


@app.route('/initSimulation', methods=['GET'])
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
        '127.0.0.1', 2000, 0.05)

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
                                     0.05,
                                     host=None,
                                     port=None,
                                     sumo_gui=False,
                                     client_order=1)

    print("done sumo init")
    # ---------------
    # synchronization
    # ---------------
    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, 'carla',
                                                False, False)

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

        # --------------
        # Spawn vehicles
        # --------------
        # Spawns sumo NPC vehicles.
        sumo_edges = sumo_net.getEdges()

        for i in range(5):
            type_id = random.choice(blueprints)
            vclass = vtypes[type_id]['vClass']

            allowed_edges = [e for e in sumo_edges if e.allows(vclass)]
            if allowed_edges:
                edge = random.choice(allowed_edges)

                traci.route.add('route_{}'.format(i), [edge.getID()])
                traci.vehicle.add('sumo_{}'.format(
                    i), 'route_{}'.format(i), typeID=type_id)
            else:
                logging.error(
                    'Could not found a route for %s. No vehicle will be spawned in sumo',
                    type_id)

        while True:
            lock.acquire()
            start = time.time()

            synchronization.tick()

            # Updates vehicle routes
            for vehicle_id in traci.vehicle.getIDList():
                route = traci.vehicle.getRoute(vehicle_id)
                index = traci.vehicle.getRouteIndex(vehicle_id)
                vclass = traci.vehicle.getVehicleClass(vehicle_id)

                if index == (len(route) - 1):
                    current_edge = sumo_net.getEdge(route[index])
                    available_edges = list(
                        current_edge.getAllowedOutgoing(vclass).keys())
                    if available_edges:
                        next_edge = random.choice(available_edges)

                        new_route = [current_edge.getID(), next_edge.getID()]
                        traci.vehicle.setRoute(vehicle_id, new_route)

            end = time.time()
            elapsed = end - start
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)
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


@app.route('/traffic_light', methods=['POST'])
def handle_traffic_light_data():
    # get the data from the request body
    data = request.get_json()
    print(data)
    # do something with the data (e.g. store it in a database, log it, etc.)

    # return a response
    return 'Received traffic light data!'


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


@app.route('/insertCar', methods=["GET"])
async def insertCar():
    lock.acquire()
    edge_list = traci.edge.getIDList()
    lock.release()
    blueprints = await vTypes()
    lock.acquire()
    with open('data/vtypes.json') as f:
        vtypes = json.load(f)['carla_blueprints']
    print(blueprints)
    # print(vtypes)
    type_id = random.choice(blueprints)
    vclass = vtypes[type_id]['vClass']
    allowed_edges = [e for e in edge_list if e.allows(vclass)]
    if allowed_edges:
        edge = random.choice(allowed_edges)
        traci.route.add('route_{}'.format(1), [edge.getID()])
        traci.vehicle.add('sumo_{}'.format(
            1), 'route_{}'.format(1), typeID=type_id)
        lock.release()
        return "inserted car"
    else:
        lock.release()
        return 'Could not found a valid  route . No vehicle will be spawned in sumo'


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
        world.set_weather(weather)
    finally:
        print("wheater set ")
    return "wheater set"


# @app.route('/insert', methods=["GET"])
# def insert():
#     print("dadas")
#     try:
#         world = client.get_world()
#         weather = world.get_weather()
#         weather.cloudiness = data['cloudcover']
#         weather.wetness = data['humidity']
#         weather.precipitation = data['precipitation']
#         world.set_weather(weather)
#     finally:
#         print("wheater set ")
#     return "wheater set"


@app.route('/testCameras', methods=["GET"])
def cam():
    try:
        vehicle_id = 1
        vehicle = world.get_actor(vehicle_id)

        # Spawn a camera and attach it to the vehicle
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform)
        spectator = world.get_spectator()
        spectator.set_transform(camera_transform)
        camera.attach_to(spectator)

        camera.listen(process_image)

    finally:
        print("wheater set ")
    return "wheater set"


def process_image(image):
    # Do something with the image
    image.save_to_disk('output_camera.png')


if __name__ == "__main__":
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)
        world = client.get_world()

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
            '127.0.0.1', 2000, 0.05)

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
        viewsettings_file = os.path.join(
            basedir, 'examples', 'viewsettings.xml')
        write_sumocfg_xml(cfg_file, net_file, vtypes_file,
                          viewsettings_file, 0)

        sumo_net = sumolib.net.readNet(net_file)

        sumo_simulation = SumoSimulation(cfg_file,
                                         0.05,
                                         host=None,
                                         port=None,
                                         sumo_gui=False,
                                         client_order=1)

        print("done sumo init")
        # ---------------
        # synchronization
        # ---------------
        synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, 'carla',
                                                    False, False)

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
        app.run(host='0.0.0.0')

    finally:
        print("end of simulation")
