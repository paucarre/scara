import logging
#from robotcontroller.logsetup import logcfg
from datetime import timedelta
from thespian.actors import *
from robotcontroller.ik import IkSolver, RobotForwardKinematics
from robotcontroller.kinematics import RobotTopology, RobotState
from robotcom.robot import RobotCommunication
from robotcom.robot_simulation import SimulationRobotCommunication
import numpy as np
import time
from enum import Enum, auto
from multiprocessing import Process, Queue
import requests
from threading import Thread
import sys
import traceback
import math
import struct
import base64
import time

def get_robot_communication(is_real):
    if is_real:
        print('Setting Up Controller')
        robot_communication = RobotCommunication()
        robot_communication.setup()
        print('Controller Set Up')
        return robot_communication
    else:
        # Simulation
        robot_communication = SimulationRobotCommunication([200, math.pi / 4, 0, -math.pi /4])
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
        'topology': robot_forward_kinamatics.robot_topology.__dict__,
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

class StateEstimationRobotState(Enum):
    INITIAL = auto()
    MOVING_TO_RIGHT = auto()
    MOVED_TO_RIGHT = auto()
    MOVING_TO_LEFT = auto()
    MOVED_TO_LEFT = auto()
    MOVING_TO_TARGET = auto()
    MOVED_TO_TARGET = auto()

global estate_estimation_robot_state, target_steps, current_parameters
estate_estimation_robot_state = StateEstimationRobotState.INITIAL
target_steps = None

def to_radians(degrees):
    return degrees * np.pi / 180


class StateEstimate():

    def __init__(self, translation, direction):
        self.translation = translation
        self.direction = direction

def request_state_estimate(parameters):
    time.sleep(1)
    robot_state = RobotState.from_robot_parameters(*parameters)
    endpoint = 'http://192.168.0.39:7000/message'
    try:
        response = requests.post(endpoint , json=robot_state.to_dictionary())
        response = response.json()
        if 'translation' in response:
            translation = np.array(response['translation'])
            print('translation', translation)
            translation[2] += 100 # safety guard
            translation[2] = 10
            rotation = response['rotation']
            approaching_direction = rotation @ np.array([1, 0, 0]).T
            approaching_direction[2] = 0.
            approaching_direction = approaching_direction / np.linalg.norm(approaching_direction)
            translation = translation - (approaching_direction * 70)
            print('\tTranslation', response['translation'])
            print('\tAngle Z', response['angle_z'])
            return StateEstimate(translation, approaching_direction)
        return None
    except Exception as error:
        error = traceback.format_exc()
        print(error)
        print(f"Error trying to connect to '{endpoint}'")
        return None
    #thread = Thread(target=publish_data)
    #thread.daemon = True
    #thread.start()


def inverse_kinematics(estate_estimate):
    data = {
        'x': estate_estimate.translation[0],
        'y': estate_estimate.translation[1],
        'z': estate_estimate.translation[2],
        'dx': estate_estimate.direction[0],
        'dy': estate_estimate.direction[1]
    }
    response = requests.post('http://localhost:8000/inverse_kinematics', json = data, verify = False)
    if response.status_code == requests.codes.ok:
        solution = response.json()
        angle_solution = solution['angle_solution']
        parameters = [ data['z'] ] + angle_solution
        return parameters
    else:
        print('NO IK SOLUTION FOUND')
        return None

def control_loop(task_queue, robot_communication):
    global estate_estimation_robot_state, target_steps, current_parameters
    endpoint='http://127.0.0.1:5000/state_updated'
    robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=240, angle_wide_1=180, angle_wide_2=180 + 90, angle_wide_3=180 + 90)
    robot_forward_kinamatics = RobotForwardKinematics(robot_topology)
    while(True):
        try:
            current_steps = robot_communication.get_steps()
            current_parameters = robot_communication.steps_to_parameters(current_steps)
            target_parameter = robot_communication.steps_to_parameters(robot_communication.get_target_steps())
            state_data = get_state_data(robot_forward_kinamatics, \
                RobotState.from_robot_parameters(*current_parameters), \
                RobotState.from_robot_parameters(*target_parameter))
            publish(state_data, endpoint)
            state_estimate = None

            if estate_estimation_robot_state == StateEstimationRobotState.INITIAL:
                target_steps = robot_communication.move_to_parameters([250., to_radians(86.7079), to_radians(49.9155), to_radians(315)])
                estate_estimation_robot_state = StateEstimationRobotState.MOVING_TO_RIGHT
            elif estate_estimation_robot_state == StateEstimationRobotState.MOVING_TO_RIGHT:
                if current_steps == target_steps:
                    estate_estimation_robot_state = StateEstimationRobotState.MOVED_TO_RIGHT
                    state_estimate = request_state_estimate(current_parameters)
            elif estate_estimation_robot_state == StateEstimationRobotState.MOVED_TO_RIGHT:
                target_steps = robot_communication.move_to_parameters([250., to_radians(310.0845), to_radians(273.2921), to_radians(45)])
                estate_estimation_robot_state = StateEstimationRobotState.MOVING_TO_LEFT
            elif estate_estimation_robot_state == StateEstimationRobotState.MOVING_TO_LEFT:
                if current_steps == target_steps:
                    estate_estimation_robot_state = StateEstimationRobotState.INITIAL
                    state_estimate = request_state_estimate(current_parameters)
            elif estate_estimation_robot_state == StateEstimationRobotState.MOVING_TO_TARGET:
                if current_steps == target_steps:
                    state_estimate = request_state_estimate(current_parameters)
                    estate_estimation_robot_state = StateEstimationRobotState.INITIAL
            if estate_estimation_robot_state is not StateEstimationRobotState.MOVING_TO_TARGET and state_estimate is not None:
                parameters = inverse_kinematics(state_estimate)
                if parameters is not None:
                    target_steps = robot_communication.move_to_parameters(parameters)
                    estate_estimation_robot_state = StateEstimationRobotState.MOVING_TO_TARGET

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
        except Exception as error:
            error = traceback.format_exc()
            print('ERRROR!', error)

def control(task_queue, robot_communication):
    control_process = Process(target=control_loop, args=(task_queue, robot_communication, ))
    control_process.start()
    return control_process