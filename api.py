from flask import Flask, jsonify, request
from flask import Flask
import traci
import threading
import random
import carla 
import json
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
    traci.init(58505)
    traci.setOrder(2)
    simulation_thread = threading.Thread(target=run_simulation)
    simulation_thread.start()
    return 'Simulation Initialized'

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
        vehicle_info[v_id] = {"position": pos, "speed": speed,"laneID":laneid,"typeid":typeid}
    lock.release()    
    return jsonify(vehicle_info)

@app.route('/insertCart', methods=["POST"])
def insertCar():
    lock.acquire()
    data = request.get_json()
    traci.vehicle.add('sumo_{}'.format(12), 'route_{}'.format(1), typeID='vehicle.bh.crossbike')
    print(data)
    lock.release()    
    return ""

@app.route('/set-weather', methods=["POST"])
def setWeather():
    data = request.get_json()
    print((data))
    try:
        client = carla.Client('localhost', 3000)
        client.set_timeout(30.0)
        world = client.get_world()
        weather = world.get_weather()
        weather.cloudiness  = data['cloudcover']
        weather.wetness  = data['humidity']
        weather.precipitation = data['precipitation']
        world.set_weather(weather)
    finally:
        print("wheater set ")
    return "wheater set"




if __name__ == "__main__":
    app.run(host='0.0.0.0')





