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
robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180 + 90, angle_wide_3=180 + 90)
global ik_solver

print('Loading IK')
ik_solver = IkSolver(robot_topology)
print('IK loaded')

@app.route('/inverse_kinematics', methods=['POST'])
def inverse_kinematics():
    global ik_solver
    content = request.json
    solutions = []
    solutions = ik_solver.compute_constrained_ik(dx=content['dx'], dy=content['dy'], x=content['x'], y=content['y'])
    if len(solutions) > 0:
        return jsonify(solutions[0].__dict__)
    else:
        return 'NO IK SOLUTION FOUND', 404
    '''
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
    '''


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', debug=True, port=7000)