import logging
#from robotcontroller.logsetup import logcfg
from datetime import timedelta
from thespian.actors import *
from robotcontroller.ik import IkSolver, RobotForwardKinematics
from robotcontroller.kinematics import RobotTopology, RobotState
from robotcom.robot import RobotCommunication
import numpy as np
import time
from multiprocessing import Process, Queue
import requests
from threading import Thread
import sys
import traceback

def get_robot_communication():
    print('Setting Up Controller')
    robot_communication = RobotCommunication()
    robot_communication.setup()
    print('Controller Set Up')
    return robot_communication

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


def get_state_data(robot_forward_kinamatics, current_parameters, target_parameter):
    robot_transformation = robot_forward_kinamatics.get_transformation(current_parameters)
    robot_end_effector = robot_transformation @ np.array([0, 0, 0, 1])
    robot_transformations_current = robot_forward_kinamatics.get_transformations(current_parameters)
    robot_transformations_target = robot_forward_kinamatics.get_transformations(target_parameter)
    current_cartesian_state = [(transformation @ np.array([0, 0, 0, 1])).tolist() for transformation in robot_transformations_current]
    target_cartesian_state = [(transformation @ np.array([0, 0, 0, 1])).tolist() for transformation in robot_transformations_target]
    data = {
        'current_state': {
            'parameters': current_parameters.__dict__,
            'cartesian': current_cartesian_state
        },
        'target_state': {
            'parameters': target_parameter.__dict__,
            'cartesian': target_cartesian_state
        }
    }
    return data

def control_loop(task_queue, robot_communication):
    endpoint='http://127.0.0.1:5000/state_updated'
    robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180 + 90, angle_wide_3=180 + 90)
    robot_forward_kinamatics = RobotForwardKinematics(robot_topology)
    while(True):
        current_parameters = robot_communication.steps_to_parameters(robot_communication.get_steps())
        target_parameter =robot_communication.steps_to_parameters(robot_communication.get_target_steps())
        state_data = get_state_data(robot_forward_kinamatics, RobotState(*current_parameters), RobotState(*target_parameter))
        publish(state_data, endpoint)
        if not task_queue.empty():
            task = task_queue.get()
            print('task', task)
            if task['type'] == 'MOVE_TO_STEPS':
                robot_communication.move_to_steps(task['steps'])
            if task['type'] == 'MOVE_TO_PARAMETERS':
                robot_communication.move_to_parameters(task['parameters'])
            elif task['type'] == 'GET_STEPS':
                steps = steps = robot_communication.get_steps()
                message = { 'type': 'GET_STEPS_RESPONSE', 'steps': steps}

def control(task_queue):
    robot_communication = get_robot_communication()
    control_process = Process(target=control_loop, args=(task_queue, robot_communication, ))
    control_process.start()
    return control_process