from flask import Flask, request, send_from_directory,  Response, render_template, jsonify
from robotcontroller.ik import IkSolver
from robotcontroller.kinematics import RobotTopology
from robotcontroller.ikcontroller import IkController
from multiprocessing import Process, Manager
import numpy as np
from flask import jsonify

import logging
import json, os
from threading import Thread
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.config['SESSION_TYPE'] = 'filesystem'
app.config['TEMPLATES_AUTO_RELOAD'] = True
socketio = SocketIO(app, async_mode=None)
robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180 + 90, angle_wide_3=180 + 90)
global ik_controller
print('Loading IK Controller')
ik_controller = IkController(robot_topology)
print('IK Controller loaded')
print('Setting Up IK Controller')
ik_controller.setup()
print('IK Controller Set Up')

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
    angles = [angle_1, angle_2, angle_3]
    angles = [ (angle / 180.0) * np.pi for angle in angles]
    positions = [linear_1] + angles
    ik_controller.move(positions)
    return 'OK', 200

@app.route('/inverse_kinematics', methods=['POST'])
def inverse_kinematics():
    global ik_controller
    x = float(request.form['x'])
    y = float(request.form['y'])
    z = float(request.form['z'])
    dx = float(request.form['dx'])
    dy = float(request.form['dy'])
    solutions = []
    solutions = ik_controller.ik_solver.compute_constrained_ik(dx=dx, dy=dy, x=x, y=y)
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