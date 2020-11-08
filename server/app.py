from flask import Flask, request, send_from_directory,  Response, render_template, jsonify
from robotcontroller.ik import IkSolver
from robotcontroller.kinematics import RobotTopology
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
    data = {
        'x': float(request.form['x']),
        'y': float(request.form['y']),
        'z': float(request.form['z']),
        'dx': float(request.form['dx']),
        'dy': float(request.form['dy'])
    }
    response = requests.post('http://localhost:7000/inverse_kinematics', json = data, verify = False)
    if response.status_code == requests.codes.ok:
        solution = response.json()
        angle_solution = solution['angle_solution']
        ui_solution = {}
        for angle_idx in range(len(angle_solution)):
            current_solution = round( angle_solution[angle_idx] * 180.0 / np.pi, 4 )
            if current_solution < 0.0:
                current_solution += 360
            ui_solution[f'angle_{angle_idx + 1}'] = current_solution
        ui_solution['linear_1'] = data['z']
        ui_solution['solution_type'] = solution['solution_type']
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