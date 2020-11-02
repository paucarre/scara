from flask import Flask, request, send_from_directory,  Response, render_template, jsonify
from robotcontroller.ik import IkSolver
from robotcontroller.kinematics import RobotTopology
from robotcontroller.ikcontroller import IkController
from multiprocessing import Process, Manager
import numpy as np
from flask import jsonify
from datetime import timedelta
import sys
import logging
import json, os
from threading import Thread
from flask_socketio import SocketIO, emit
import traceback
import requests

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.config['SESSION_TYPE'] = 'filesystem'
app.config['TEMPLATES_AUTO_RELOAD'] = True
socketio = SocketIO(app, async_mode=None)
robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180 + 90, angle_wide_3=180 + 90)
global ik_solver

print('Loading IK')
ik_solver = IkSolver(robot_topology)
print('IK loaded')

@socketio.on('connect')
def test_connect():
    print('Client connected')

@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')

def publish(data, endpoint):
    if data is not None:
        def publish_data():
            try:
                resp = requests.post(endpoint, json = data, verify = False)
            except:
                print(f"Error trying to connect to '{endpoint}'", file=sys.stderr)
                print(traceback.format_exc(), file=sys.stderr)
        thread = Thread(target=publish_data)
        thread.daemon = True
        thread.start()

@app.route('/write_target_state', methods=['POST'])
def write_target_state():
    linear_1 = float(request.form['linear_1'])
    angle_1 = float(request.form['angle_1'])
    angle_2 = float(request.form['angle_2'])
    angle_3 = float(request.form['angle_3'])
    angles = [angle_1, angle_2, angle_3]
    angles = [ (angle / 180.0) * np.pi for angle in angles]
    parameters = [linear_1] + angles
    move_message = { 'type': 'MOVE_TO_PARAMETERS', 'parameters': parameters }
    publish(move_message, 'http://localhost:6000/message')
    return 'OK', 200

@app.route('/inverse_kinematics', methods=['POST'])
def inverse_kinematics():
    global ik_solver
    x = float(request.form['x'])
    y = float(request.form['y'])
    z = float(request.form['z'])
    dx = float(request.form['dx'])
    dy = float(request.form['dy'])
    solutions = []
    solutions = ik_solver.compute_constrained_ik(dx=dx, dy=dy, x=x, y=y)
    if len(solutions) > 0:
        print(solutions)
        ui_solution = {}
        angle_solution = solutions[0].angle_solution
        for angle_idx in range(len(angle_solution)):
            solution = round( angle_solution[angle_idx] * 180.0 / np.pi, 4 )
            if solution < 0.0:
                solution += 360
            ui_solution[f'angle_{angle_idx + 1}'] = solution
        ui_solution['linear_1'] = z
        ui_solution['solution_type'] = solutions[0].solution_type
        return jsonify(ui_solution)
    else:
        return 'NO IK SOLUTION FOUND', 404

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