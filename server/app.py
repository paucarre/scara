from flask import Flask, request, send_from_directory,  Response, render_template, jsonify
from ik import IkSolver
from kinematics import RobotTopology
import numpy as np
from flask import jsonify

import logging
import json, os
from threading import Thread
from flask_socketio import SocketIO, emit

from arduino_connector import ArduinoConnector

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.config['SESSION_TYPE'] = 'filesystem'
app.config['TEMPLATES_AUTO_RELOAD'] = True
socketio = SocketIO(app, async_mode=None)
robot_topology = RobotTopology(l1=142.5, l2=142.5, l3=142.5 + 19.0, h1=290, 
    angle_wide_1=248, angle_wide_2=248, angle_wide_3=248)
global ik_solver
ik_solver = None

def init_ik_solver():
    global ik_solver
    def multithreaded_ik_solver_init():
        global ik_solver
        if ik_solver is None:
            ik_solver = IkSolver(robot_topology)
    thread = Thread(target=multithreaded_ik_solver_init)
    thread.daemon = True
    thread.start()

init_ik_solver()

@socketio.on('connect')
def test_connect():
    print('Client connected')

@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')

@app.route('/write_target_state', methods=['POST'])
def write_target_state():
    linear_1 = float(request.form['linear_1'])
    angle_1 = float(request.form['angle_1'])
    angle_2 = float(request.form['angle_2'])
    angle_3 = float(request.form['angle_3'])
    arduino_connector = ArduinoConnector('/dev/ttyUSB0')
    arduino_connector.write_target_state([linear_1, angle_1, angle_2, angle_3])
    return 'OK', 200

@app.route('/inverse_kinematics', methods=['POST'])
def inverse_kinematics():
    global ik_solver
    x = float(request.form['x'])
    y = float(request.form['y'])
    z = float(request.form['z'])
    dx = float(request.form['dx'])
    dy = float(request.form['dy'])
    solutions = ik_solver.compute_constrained_ik(dx=dx, dy=dy, x=x, y=y)
    if len(solutions) > 0:
        print(solutions)
        angle_solution = solutions[0].angle_solution
        for angle in angle_solution:
            angle_solution[angle] = round( angle_solution[angle] * 180.0 / np.pi, 4 )
            if angle_solution[angle] < 0.0:
                angle_solution[angle] += 360
        angle_solution['linear_1'] = z
        angle_solution['solution_type'] = solutions[0].solution_type
        return jsonify(solutions[0].angle_solution)
    else:
        return 'OK', 404

@app.route('/state_updated', methods=['POST'])
def state_updated():
    data = request.get_json()
    socketio.emit('state_updated', data)
    return 'OK', 200

@app.route('/camera_updated', methods=['POST'])
def camera_updated():
    data = request.get_json()
    socketio.emit('camera_updated', data)
    return 'OK', 200

@app.route('/svg/<path:path>')
def send_svg(path):
    return send_from_directory('svg_files', path)

@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', debug=True, port=5000)