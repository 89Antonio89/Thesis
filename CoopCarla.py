import carla
import os
import io
import argparse
import json
import logging
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

    ET.SubElement(root, 'num-clients', {'value': str(additional_traci_clients+1)})

    tree = ET.ElementTree(root)
    tree.write(cfg_file, pretty_print=True, encoding='UTF-8', xml_declaration=True)



def on_traffic_light_event(event, traffic_light):
    print("light triggered")
    if event.current_state == carla.TrafficLightState.Green:
        # Check if a vehicle is passing by the traffic light
        vehicles = world.get_actors().filter('vehicle.*')
        for vehicle in vehicles:
            vehicle_location = vehicle.get_location()
            traffic_light_location = traffic_light.get_location()
            distance = vehicle_location.distance(traffic_light_location)
            if distance < 50:  # Adjust this value based on your needs
                # Send the car passing event to your API
                requests.post('http://127.0.0.1:5000/traffic_light', json={'event': 'car_passed', 'traffic_light_id': traffic_light.id})

def main():
# Connect to the Carla server and create a Carla world object
    try:
        client = carla.Client('localhost', args.port)
        client.set_timeout(30.0)
        world = client.get_world()
    finally:
        print("connected")
            


    # Read file content
    xodr_file_path = 'boavista.xodr'
    with  io.open(xodr_file_path, 'r',encoding="utf-8") as f:
     xodr_content = f.read()

    print("loaded file")
    
    vertex_distance = 2.0  # in meters
    max_road_length = 50.0 # in meters
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
    print("loaded map")
 # Temporal folder to save intermediate files.
    tmpdir = tempfile.mkdtemp()

    # ----------------
    # carla simulation
    # ----------------
    carla_simulation = CarlaSimulation(args.host, args.port, args.step_length)

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
    write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, args.additional_traci_clients)

    sumo_net = sumolib.net.readNet(net_file)
    sumo_simulation = SumoSimulation(cfg_file,
                                     args.step_length,
                                     host=args.sumo_host,
                                     port=args.sumo_port,
                                     sumo_gui=args.sumo_gui,
                                     client_order=args.client_order)
    # ---------------
    # synchronization
    # ---------------
    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)
    
    try:
        # ----------
        # Blueprints
        # ----------
        with open('data/vtypes.json') as f:
            vtypes = json.load(f)['carla_blueprints']

        blueprints = vtypes.keys()

        filterv = re.compile(args.filterv)
        blueprints = list(filter(filterv.search, blueprints))

        if args.safe:
            blueprints = [
                x for x in blueprints if vtypes[x]['vClass'] not in ('motorcycle', 'bicycle')
            ]
            blueprints = [x for x in blueprints if not x.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.endswith('t2')]
            blueprints = [x for x in blueprints if not x.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.endswith('ambulance')]

        if not blueprints:
            raise RuntimeError('No blueprints available due to user restrictions.')

        if args.number_of_walkers > 0:
            logging.warning('Pedestrians are not supported yet. No walkers will be spawned.')
       
       
        traffic_lights = world.get_actors().filter('traffic.traffic_light') 
        print(traffic_lights)   
        traffic_light_sensors = []
        for traffic_light in traffic_lights:
            collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
            collision_sensor_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
            collision_sensor = world.spawn_actor(collision_sensor_bp, collision_sensor_transform, attach_to=traffic_light)
            collision_sensor.listen(lambda event: on_traffic_light_event(event, traffic_light))
            traffic_light_sensors.append(collision_sensor)
        print('atached sensors')
        for v in world.get_actors().filter('*vehicle*'):
            collision_sensor_bp = world.get_blueprint_library().find('sensor.other.collision')
            collision_sensor_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
            collision_sensor = world.spawn_actor(collision_sensor_bp, collision_sensor_transform, attach_to=v)
        # --------------
        # Spawn vehicles
        # --------------
        # Spawns sumo NPC vehicles.
        sumo_edges = sumo_net.getEdges()

        for i in range(args.number_of_vehicles):
            type_id = random.choice(blueprints)
            vclass = vtypes[type_id]['vClass']

            allowed_edges = [e for e in sumo_edges if e.allows(vclass)]
            if allowed_edges:
                edge = random.choice(allowed_edges)

                traci.route.add('route_{}'.format(i), [edge.getID()])
                traci.vehicle.add('sumo_{}'.format(i), 'route_{}'.format(i), typeID=type_id)
            else:
                logging.error(
                    'Could not found a route for %s. No vehicle will be spawned in sumo',
                    type_id)

        while True:
            start = time.time()

            synchronization.tick()

            # Updates vehicle routes
            for vehicle_id in traci.vehicle.getIDList():
                route = traci.vehicle.getRoute(vehicle_id)
                index = traci.vehicle.getRouteIndex(vehicle_id)
                vclass = traci.vehicle.getVehicleClass(vehicle_id)

                if index == (len(route) - 1):
                    current_edge = sumo_net.getEdge(route[index])
                    available_edges = list(current_edge.getAllowedOutgoing(vclass).keys())
                    if available_edges:
                        next_edge = random.choice(available_edges)

                        new_route = [current_edge.getID(), next_edge.getID()]
                        traci.vehicle.setRoute(vehicle_id, new_route)
                 
            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)
                
        
        
        

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        synchronization.close()

        if os.path.exists(tmpdir):
            shutil.rmtree(tmpdir)
    




















if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p',
                           '--port',
                           metavar='P',
                           default=3000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           default=None,
                           help='IP of the sumo host server (default: None)')
    argparser.add_argument('--sumo-port',
                           default=3002,
                           type=int,
                           help='TCP port to listen to (default: None)')
    argparser.add_argument('-n',
                           '--number-of-vehicles',
                           metavar='N',
                           default=10,
                           type=int,
                           help='number of vehicles (default: 10)')
    argparser.add_argument('-w',
                           '--number-of-walkers',
                           metavar='W',
                           default=0,
                           type=int,
                           help='number of walkers (default: 0)')
    argparser.add_argument('--safe',
                           action='store_true',
                           help='avoid spawning vehicles prone to accidents')
    argparser.add_argument('--filterv',
                           metavar='PATTERN',
                           default='vehicle.*',
                           help='vehicles filter (default: "vehicle.*")')
    argparser.add_argument('--filterw',
                           metavar='PATTERN',
                           default='walker.pedestrian.*',
                           help='pedestrians filter (default: "walker.pedestrian.*")')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--additional-traci-clients',
                           metavar='TRACI_CLIENTS',
                           default=0,
                           type=int,
                           help='number of additional TraCI clients to wait for (default: 0)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    args = argparser.parse_args()
    main()



