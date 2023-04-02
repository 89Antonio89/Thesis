from flask import Flask, jsonify, request
from sumolib import checkBinary
import traci
import time

import subprocess
app = Flask(__name__)



    
@app.route('/simulation', methods=['GET'])
def simulation_status():
    pos = traci.vehicle.getPosition("vehicle_1")
    speed = traci.vehicle.getSpeed("vehicle_1")
    # code to interact with the simulation using the TraCI library
    return jsonify({"vehicle_1": {"position": pos, "speed": speed}})



if __name__ == '__main__':
    app.run(debug=True)