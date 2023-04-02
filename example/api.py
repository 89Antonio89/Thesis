from flask import Flask, jsonify, request
from flask import Flask
import traci
import threading
import random

app = Flask(__name__)

# Create the lock
lock = threading.Lock()
condition = threading.Condition(lock)
steps = 10000
simulation_running = True

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
        
        
@app.route('/init', methods=['GET'])
def init_simulation():
    traci.init(3000)
    traci.setOrder(2)
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.start()
    return 'Simulation Initialized'


def create_routes_file(num_vehicles, output_file):
    edges = traci.edge.getIDList()
    with open(output_file, 'w') as f:
        f.write('<routes>\n')
        f.write('    <vType id="car" vClass="passenger"/>\n')
        for i in range(num_vehicles):
            origin = random.choice(edges)
            destination = random.choice(edges)
            while destination == origin:
                destination = random.choice(edges)
            route = traci.simulation.findRoute(origin, destination)
            if route:
                route_str = ' '.join(route.edges)
                f.write(f'    <route id="route{i}" edges="{route_str}"/>\n')
                f.write(f'    <vehicle id="car{i}" type="car" route="route{i}" depart="{i}"/>\n')
        f.write('</routes>')
    traci.close()

@app.route("/simulation")
def simulation():
    # Use traci commands here to interact with the SUMO simulation
    return jsonify({"status": "success"})

@app.route('/simulation1', methods=["GET"])
def simulation_status():
    lock.acquire()
    pos = traci.vehicle.getPosition("1")
    speed = traci.vehicle.getSpeed("1")
    lock.release()
    # code to interact with the simulation using the TraCI library
    return jsonify({"1": {"position": pos, "speed": speed}})

@app.route('/carsInfo', methods=["GET"])
def carsInfo():
    lock.acquire()
    vehicle_ids = traci.vehicle.getIDList()
    vehicle_info = {}
    for v_id in vehicle_ids:
        pos = traci.vehicle.getPosition(v_id)
        speed = traci.vehicle.getSpeed(v_id)
        vehicle_info[v_id] = {"position": pos, "speed": speed}
    lock.release()    
    return jsonify(vehicle_info)

@app.route('/add_vehicles', methods=['POST'])
def add_vehicles(): 
    lock.acquire()
    num_vehicles = int(request.json['num_vehicles'])
    route_ids = traci.route.getIDList()
    for i in range(num_vehicles):
        while True:
            # Select two random edges in the network
            edge_ids = traci.edge.getIDList()
            start_edge = random.choice(edge_ids)
            end_edge = random.choice(edge_ids)
            # Find a valid route between the two edges
            try:
                route = traci.simulation.findRoute(start_edge, end_edge)
            except traci.exceptions.TraCIException:
                continue
            # Select a random route ID from the valid route
        
            vehicle_id = f"vehicle_{i}"
            traci.vehicle.add(vehicle_id, route)
    lock.release()    
    return "Vehicles added successfully!"


def checkIfValid(edges):
    check = True
    # check if vehicles are allowed to depart from any lane of each edge in the route
    for edge in edges:
        allow = traci.edge.getMaxSpeed(edge) >0
    if not allow:
        return allow
    
    return True   
       
    
@app.route('/add_routes')    
def add_routes():
    lock.acquire()
    edge_ids = traci.edge.getIDList()
    # create a route for each edge and add it to the simulation
    for edge_id in edge_ids:
        route_id = f"{edge_id}_route"
        traci.route.add(route_id, [edge_id])
    lock.release()
    return 'Added routes'

if __name__ == "__main__":
    app.run(debug=True)





