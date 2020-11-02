from arduino_connector import ArduinoConnector
from kinematics import RobotTopology, RobotState, RobotForwardKinematics, TrackerForwardKinematics, TrackerState
from camera import load_camera_matrix, TrackerState
from ik import IkSolver
from tracker_orientation import TrackerOrientation

import traceback
import socket
import time
from imutils.video import VideoStream
import requests
import base64
import numpy as np
import io
import cv2
from PIL import Image
from multiprocessing import Pool
from threading import Thread
import sys, select
import os
import glob
import math

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

class RealtimeWebcam():

    def __init__(self, camera_index=4, endpoint='http://127.0.0.1:5000/camera_updated'):
        self.camera_index = camera_index
        self.video_capture = None
        self.endpoint = endpoint
        self.camera_matrix, self.distortion_coefficients = load_camera_matrix()
        inverse_camera_matrix = np.linalg.inv(self.camera_matrix)
        self.tracker_orientation = TrackerOrientation(inverse_camera_matrix)
        self.end_effector_tracker_state = None

    def resize(self, image, scale_percent=60):
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dimensions = (width, height)
        image = cv2.resize(image, dimensions, interpolation = cv2.INTER_AREA)
        return image

    def try_to_collect(self):
        tracker_found = False
        try:
            if self.video_capture is not None and self.video_capture.isOpened():
                success, frame = self.video_capture.read()
                if success:
                    is_success, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 30])
                    file_object = io.BytesIO(buffer)
                    file_object.seek(0)
                    camera_image = base64.b64encode(file_object.read())
                    data = { 'camera_image': camera_image }
                    return data, None
                    '''
                    frame_position_orientation = self.tracker_orientation.get_camera_orientation(frame)
                    current_end_effector_tracker_state = None
                    if frame_position_orientation is not None:
                        pan, t, error, frame = frame_position_orientation
                        #TODO: set back pan
                        current_end_effector_tracker_state = TrackerState(t[0], t[1], t[2], 0.0)
                        if self.end_effector_tracker_state is None:
                            self.end_effector_tracker_state = current_end_effector_tracker_state
                        else:
                            self.end_effector_tracker_state = current_end_effector_tracker_state
                            #(0.8 * current_end_effector_tracker_state) + \
                            #    (0.2 * self.end_effector_tracker_state)
                        tracker_found = True
                    else:
                        print("Error getting orientation and position of tracker", file=sys.stderr)
                    frame = cv2.flip(frame, 1)
                    is_success, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 15])
                    if(is_success):
                        file_object = io.BytesIO(buffer)
                        file_object.seek(0)
                        camera_image = base64.b64encode(file_object.read())
                        data = { 'camera_image': camera_image }
                        return data, self.end_effector_tracker_state if tracker_found else None
                    else:
                        print("Error encoding webcam frame", file=sys.stderr)
                    '''
                else:
                    self.reload_video_capture()
            else:
                self.reload_video_capture()
        except:
            print("Error capturing camera", file=sys.stderr)
            print(traceback.format_exc(), file=sys.stderr)
        return None, None

    def reload_video_capture(self):
        self.video_capture = cv2.VideoCapture(self.camera_index)
        self.video_capture.set(cv2.CAP_PROP_EXPOSURE, 0.25)

class StateCollector():

    def __init__(self, robot_topology, endpoint='http://127.0.0.1:5000/state_updated'):
        self.endpoint = endpoint
        self.robot_topology = robot_topology
        self.tracker_forward_kinematics = TrackerForwardKinematics()
        self.robot_forward_kinamatics = RobotForwardKinematics(robot_topology)
        self.ik_solver = IkSolver(robot_topology)
        self.follow_tracker_mode = False
        self.arduino_connector = None
        self.ik_solution = None
        self.try_to_load_arduino_connector()
        self.tracker_position_estimation = None
        self.tracker_orientation_estimation = None
        self.last_tracker_position = None
        self.last_tracker_orientation = None

    def try_to_load_arduino_connector(self):
        try:
            print("Trying to establish connection with mictrocontroller through serial...")
            self.arduino_connector = ArduinoConnector('/dev/ttyUSB0')
            print("Successfully estrablished connection with microcontroller through serial.")
        except:
            print("Error trying to connect to microcontroller though serial", file=sys.stderr)

    def collect_state(self):
        return None, None

    def try_to_collect(self, frame, end_effector_tracker_state):
        current_state, target_state = self.collect_state()
        if current_state is not None and target_state is not None:
            robot_transformation = self.robot_forward_kinamatics.get_transformation(current_state)
            robot_end_effector = robot_transformation @ np.array([0, 0, 0, 1])
            robot_transformations_current = self.robot_forward_kinamatics.get_transformations(current_state)
            robot_transformations_target = self.robot_forward_kinamatics.get_transformations(target_state)
            current_cartesian_state = [(transformation @ np.array([0, 0, 0, 1])).tolist() for transformation in robot_transformations_current]
            target_cartesian_state = [(transformation @ np.array([0, 0, 0, 1])).tolist() for transformation in robot_transformations_target]
            data = {
                'current_state': {
                    'parameters': current_state.__dict__,
                    'cartesian': current_cartesian_state
                },
                'target_state': {
                    'parameters': target_state.__dict__,
                    'cartesian': target_cartesian_state
                }
            }
            if end_effector_tracker_state is None:
                print("End effector tracker state not found", file=sys.stderr)
                if self.last_tracker_orientation is not None and self.last_tracker_position is not None:
                    self.tracker_position_estimation = (0.8 * self.tracker_position_estimation) + (0.2 * self.last_tracker_position)
                    self.tracker_orientation_estimation = (0.8 * self.tracker_orientation_estimation) + (0.2 * self.last_tracker_orientation)
            else:
                tracker_transformation = self.tracker_forward_kinematics.get_transformation(end_effector_tracker_state)
                tracker_position =  ( (robot_transformation @ tracker_transformation) @ np.array([0, 0, 0, 1]) )
                tracker_orientation = ( (robot_transformation @ tracker_transformation) @ np.array([1, 0, 0, 0]) )
                if self.tracker_position_estimation is None:
                    self.tracker_position_estimation = tracker_position
                else:
                    self.last_tracker_position = tracker_position
                    self.tracker_position_estimation = (0.8 * self.tracker_position_estimation) + (0.2 * tracker_position)
                if self.tracker_orientation_estimation is None:
                    self.tracker_orientation_estimation = tracker_orientation
                else:
                    self.last_tracker_orientation = tracker_orientation
                    self.tracker_orientation_estimation = (0.8 * self.tracker_orientation_estimation) + (0.2 * tracker_orientation)

                tracker_state = {
                    'parameters': {
                        'x': self.tracker_position_estimation[0],
                        'y': self.tracker_position_estimation[1],
                        'z': self.tracker_position_estimation[2],
                        'dx': self.tracker_orientation_estimation[0],
                        'dy': self.tracker_orientation_estimation[1]
                    }
                }
                if(self.follow_tracker_mode):
                    x_1cm_away_from_tracker = self.tracker_position_estimation[0] + (140 * self.tracker_orientation_estimation[0])
                    y_1cm_away_from_tracker = self.tracker_position_estimation[1] + (140 * self.tracker_orientation_estimation[1])
                    dx_rotated_180 = -self.tracker_orientation_estimation[0]
                    dy_rotated_180 = -self.tracker_orientation_estimation[1]
                    ik_solutions = self.ik_solver.compute_ik(dx=dx_rotated_180, dy=dy_rotated_180,
                        x=x_1cm_away_from_tracker, y=y_1cm_away_from_tracker)
                    if len(ik_solutions) > 0:
                        if self.ik_solution is None or len(ik_solutions) == 1:
                            self.ik_solution = ik_solutions[0]
                        else:
                            distance_0 = np.abs(np.array(ik_solutions[0].cartesian_solution) - \
                                np.array(self.ik_solution.cartesian_solution)).mean()
                            distance_1 = np.abs(np.array(ik_solutions[1].cartesian_solution) - \
                                np.array(self.ik_solution.cartesian_solution)).mean()
                            if(distance_0 < distance_1):
                                self.ik_solution = ik_solutions[0]
                            else:
                                self.ik_solution = ik_solutions[1]
                        if self.arduino_connector is not None:
                            height = min( max( self.tracker_position_estimation[2], 5.0 ), robot_topology.h1 - 5.0 )
                            self.arduino_connector.write_target_state([height,
                                ( 180.0 * self.ik_solution.angle_solution['angle_1'] ) / np.pi,
                                ( 180.0 * self.ik_solution.angle_solution['angle_2'] ) / np.pi,
                                ( 180.0 * self.ik_solution.angle_solution['angle_3'] ) / np.pi])


                data['tracker_state'] = tracker_state
            return data
        return None

class RandomStateCollector(StateCollector):

    def __init__(self, robot_topology, endpoint='http://127.0.0.1:5000/state_updated'):
        super().__init__(robot_topology, endpoint)

    def collect_state(self):
        current_state = RobotState(
            linear_1 = 220,
            angle_1 = 1.0 ,
            angle_2 =  1.0,
            angle_3 =  -1.0)
        if self.ik_solution is not None:
            target_state = RobotState(
                linear_1 = 220,
                angle_1 = self.ik_solution.angle_solution['angle_1'],
                angle_2 = self.ik_solution.angle_solution['angle_2'],
                angle_3 = self.ik_solution.angle_solution['angle_3'])
        else:
            target_state = current_state
        return current_state, target_state


class ArduinoStateCollector(StateCollector):

    def __init__(self, robot_topology, endpoint='http://127.0.0.1:5000/state_updated'):
        super().__init__(robot_topology, endpoint)

    def collect_state(self):
        if self.arduino_connector is None:
            self.try_to_load_arduino_connector()
        if self.arduino_connector is not None:
            print("About to read current state...")
            current_state = self.arduino_connector.read_current_state()
            print("About to read target state...")
            target_state = self.arduino_connector.read_target_state()
            return current_state, target_state
        return None, None

robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180 + 90, angle_wide_3=180 + 90)

webcam_collector = RealtimeWebcam(4)
#webcam_collector = RealtimeWebcam(f'/home/rusalka/Videos/Webcam/test.webm')
# state_collector = RandomStateCollector(robot_topology)
state_collector = ArduinoStateCollector(robot_topology)
quit = False
while(not quit):
    image_data, end_effector_tracker_state = webcam_collector.try_to_collect()
    if image_data is not None:
        publish(image_data, webcam_collector.endpoint)
    state = state_collector.try_to_collect(image_data, end_effector_tracker_state)
    if state is not None:
        publish(state, state_collector.endpoint)

    input_key, _, _ = select.select( [sys.stdin], [], [], 0 )
    #time.sleep(0.5)
    if input_key:
        input_key = sys.stdin.read(1)
        quit = quit or input_key == 'q'