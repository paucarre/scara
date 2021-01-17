import numpy as np
import modern_robotics as mr

import math
import base64
import struct

class RobotTopology():

    def __init__(self, l1, l2, l3, h1, angle_wide_1, angle_wide_2, angle_wide_3):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.h1 = h1
        self.angle_wide_1 = angle_wide_1
        self.angle_wide_2 = angle_wide_2
        self.angle_wide_3 = angle_wide_3

    def __repr__(self):
        return str(self.__dict__)

class RobotState():

    @staticmethod
    def from_robot_parameters(linear_1, angle_1, angle_2, angle_3):
        linear_1_rectified = linear_1
        angle_1_rectified = angle_1
        angle_2_rectified = angle_2 - angle_1
        angle_3_rectified = angle_3 - angle_2
        return RobotState(linear_1_rectified, angle_1_rectified, angle_2_rectified, angle_3_rectified)

    @staticmethod
    def from_dictionary(state_dictionary):
        return RobotState( \
                RobotState.decode_double(state_dictionary['linear_1']),
                RobotState.decode_double(state_dictionary['angle_1']),
                RobotState.decode_double(state_dictionary['angle_2']),
                RobotState.decode_double(state_dictionary['angle_3'])
            )

    def __init__(self, linear_1, angle_1, angle_2, angle_3):
        self.linear_1 = linear_1
        self.angle_1 = angle_1
        self.angle_2 = angle_2
        self.angle_3 = angle_3

    def to_theta_list(self):
        return np.array([self.linear_1, self.angle_1, self.angle_2, self.angle_3])

    def __repr__(self):
        return str(self.__dict__)

    @staticmethod
    def encode_double(data):
        data = bytearray(struct.pack("d", data))
        data = base64.b64encode(data)
        return data

    @staticmethod
    def decode_double(data):
        data = base64.b64decode(data)
        data = struct.unpack("d", data)[0]
        return data

    def to_dictionary(self):
        linear_1 = RobotState.encode_double(self.linear_1)
        angle_1  = RobotState.encode_double(self.angle_1)
        angle_2  = RobotState.encode_double(self.angle_2)
        angle_3  = RobotState.encode_double(self.angle_3)
        state = {
                'linear_1': linear_1,
                'angle_1': angle_1,
                'angle_2': angle_2,
                'angle_3': angle_3
            }
        return state

class RobotForwardKinematics():

    def __init__(self, robot_topology):
        self.robot_topology = robot_topology
        self.screws = self.compute_screws()
        self.end_effector_at_zero_position = self.compute_end_effector_at_zero_position()
        self.end_effectors_at_zero_position = self.compute_end_effectors_at_zero_position()
        self.test()

    def compute_screws(self):
        w_linear = np.array([0, 0, 0])
        v_linear = np.array([0, 0, 1])
        w_angular_1 = np.array([0, 0, 1])
        v_angular_1 = np.array([0, 0, 0])
        w_angular_2 = np.array([0, 0, 1])
        v_angular_2 = np.array([0, -self.robot_topology.l1, 0])
        w_angular_3 = np.array([0, 0, 1])
        v_angular_3 = np.array([0, - (self.robot_topology.l1 + self.robot_topology.l2), 0])
        screws = np.stack([
                np.concatenate( (w_linear, v_linear), axis=0),
                np.concatenate( (w_angular_1, v_angular_1), axis=0),
                np.concatenate( (w_angular_2, v_angular_2), axis=0),
                np.concatenate( (w_angular_3, v_angular_3), axis=0)], axis=0).T
        return screws

    def compute_end_effector_at_zero_position(self):
        return np.array([
                [1, 0, 0, self.robot_topology.l1 + self.robot_topology.l2 + self.robot_topology.l3],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

    def compute_end_effectors_at_zero_position(self):
        return [ np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
                np.array([
                    [1, 0, 0, self.robot_topology.l1],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
                np.array([
                    [1, 0, 0, self.robot_topology.l1 + self.robot_topology.l2],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]),
                np.array([
                    [1, 0, 0, self.robot_topology.l1 + self.robot_topology.l2 + self.robot_topology.l3],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]) ]

    def get_transformations(self, robot_configuration):
        transformations = []
        for screw_index in range(0, self.screws.shape[1]):
            transformation = mr.FKinSpace(self.end_effectors_at_zero_position[screw_index],
                                self.screws[:, 0 : screw_index + 1],
                                robot_configuration.to_theta_list()[0 : screw_index + 1])
            transformations.append(transformation)
        return transformations

    def get_transformation(self, robot_configuration):
        return mr.FKinSpace(self.end_effector_at_zero_position,
                            self.screws,
                            robot_configuration.to_theta_list())

    def test(self):
        original_topology = self.robot_topology
        original_screws = self.screws
        original_end_effector_at_zero_position = self.end_effector_at_zero_position

        self.robot_topology = RobotTopology(l1=5, l2=10, l3=15, h1=30, angle_wide_1=180, angle_wide_2=180, angle_wide_3=180)
        self.screws = self.compute_screws()
        self.end_effector_at_zero_position = self.compute_end_effector_at_zero_position()

        # zero configuration
        robot_configuration = RobotState.from_robot_parameters(0, 0, 0, 0)
        transformation = self.get_transformation(robot_configuration)
        position = np.array([0, 0, 0, 1])
        transformed_position = transformation @ position
        expected_transformed_position = np.array([[self.robot_topology.l1 +
            self.robot_topology.l2 + self.robot_topology.l3, 0, 0, 1]])
        error = np.linalg.norm(transformed_position - expected_transformed_position, ord='fro')
        assert error < 1e-10
        direction = np.array([0, 1, 0, 0])
        transformed_direction = transformation @ direction
        expected_transformed_direction = np.array([[0, 1, 0, 0]])
        error = np.linalg.norm(transformed_direction - expected_transformed_direction, ord='fro')
        assert error < 1e-10

        # (20, 0, 0, 90) config
        robot_configuration = RobotState.from_robot_parameters(20, 0, 0, np.pi / 2)
        transformation = self.get_transformation(robot_configuration)
        position = np.array([0, 0, 0, 1])
        transformed_position = transformation @ position
        expected_transformed_position = np.array([[self.robot_topology.l1 + self.robot_topology.l2,
            self.robot_topology.l3, robot_configuration.linear_1, 1]])
        error = np.linalg.norm(transformed_position - expected_transformed_position, ord='fro')
        assert error < 1e-10
        direction = np.array([1, 0, 0, 0])
        transformed_direction = transformation @ direction
        expected_transformed_direction = np.array([[0, 1, 0, 0]])
        error = np.linalg.norm(transformed_direction - expected_transformed_direction, ord='fro')
        assert error < 1e-10

        # (20, 0, 90, 90) config
        robot_configuration = RobotState.from_robot_parameters(20, 0, np.pi / 2, np.pi / 2 + np.pi / 2)
        transformation = self.get_transformation(robot_configuration)
        position = np.array([0, 0, 0, 1])
        transformed_position = transformation @ position
        expected_transformed_position = np.array([[self.robot_topology.l1 - self.robot_topology.l3,
            self.robot_topology.l2, robot_configuration.linear_1, 1]])
        error = np.linalg.norm(transformed_position - expected_transformed_position, ord='fro')
        assert error < 1e-10
        direction = np.array([1, 0, 0, 0])
        transformed_direction = transformation @ direction
        expected_transformed_direction = np.array([[-1, 0, 0, 0]])
        error = np.linalg.norm(transformed_direction - expected_transformed_direction, ord='fro')
        assert error < 1e-10

        # (20, -90, 90, 90) config
        '''
        robot_configuration = RobotState.from_robot_parameters(20, -np.pi / 2, np.pi / 2 -np.pi / 2, np.pi / 2 + np.pi / 2)
        transformation = self.get_transformation(robot_configuration)
        position = np.array([0, 0, 0, 1])
        transformed_position = transformation @ position
        expected_transformed_position = np.array([[
            self.robot_topology.l3 - self.robot_topology.l1, self.robot_topology.l2, robot_configuration.linear_1, 1]])
        error = np.linalg.norm(transformed_position - expected_transformed_position, ord='fro')
        assert error < 1e-10
        direction = np.array([1, 0, 0, 0])
        transformed_direction = transformation @ direction
        expected_transformed_direction = np.array([[0, 1, 0, 0]])
        error = np.linalg.norm(transformed_direction - expected_transformed_direction, ord='fro')
        assert error < 1e-10
        '''

        self.robot_topology = original_topology
        self.screws = original_screws
        self.end_effector_at_zero_position = original_end_effector_at_zero_position


class TrackerState():

    def __init__(self, x, y, z, angle_z):
        self.x = x
        self.y = y
        self.z = z
        self.angle_z = angle_z

    def to_theta_list(self):
        return np.array([self.x, self.y, self.z, self.angle_z + np.pi])

    def __rmul__(self, real):
        return TrackerState(real * self.x, real * self.y, real * self.z, real * self.angle_z)

    def __add__(self, right):
        return TrackerState(right.x + self.x, right.y + self.y, right.z + self.z, right.angle_z + self.angle_z)

    def __repr__(self):
        return str(self.__dict__)

class TrackerForwardKinematics():

    def __init__(self):
        self.screws = self.compute_screws()
        self.end_effector_at_zero_position = self.compute_end_effector_at_zero_position()
        self.test()

    def compute_screws(self):
        w_x = np.array([0, 0, 0])
        v_x = np.array([1, 0, 0])
        w_y = np.array([0, 0, 0])
        v_y = np.array([0, -1, 0])
        w_z = np.array([0, 0, 0])
        v_z = np.array([0, 0, 1])
        w_angle_z = np.array([0, 0, -1])
        v_angle_z = np.array([0, 0, 0])
        screws = np.stack([
                np.concatenate( (w_x, v_x), axis=0),
                np.concatenate( (w_y, v_y), axis=0),
                np.concatenate( (w_z, v_z), axis=0),
                np.concatenate( (w_angle_z, v_angle_z), axis=0)], axis=0).T
        return screws

    def compute_end_effector_at_zero_position(self):
        return np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

    def get_transformation(self, tracker_configuration):
        return mr.FKinSpace(self.end_effector_at_zero_position,
                            self.screws,
                            tracker_configuration.to_theta_list())

    def test(self):
        original_screws = self.screws
        original_end_effector_at_zero_position = self.end_effector_at_zero_position
        z = 10
        tracker_configuration = TrackerState(10, 20, z, 0)
        transformation = self.get_transformation(tracker_configuration)
        position = np.array([0,0,0,1])
        transformed_position = transformation @ position
        expected_transformed_position = np.array([[10, -20, z, 1]])
        error = np.linalg.norm(transformed_position - expected_transformed_position, ord='fro')
        assert error < 1e-10

        tracker_configuration = TrackerState(10, 20, z, np.pi / 2)
        transformation = self.get_transformation(tracker_configuration)
        position = np.array([0 ,0, 0, 1])
        transformed_position = transformation @ position
        expected_transformed_position = np.array([[10, -20, z, 1]])
        error = np.linalg.norm(transformed_position - expected_transformed_position, ord='fro')
        assert error < 1e-10

        tracker_configuration = TrackerState(10, 20, z, 0)
        transformation = self.get_transformation(tracker_configuration)
        direction = np.array([0, 1, 0, 0])
        transformed_direction = transformation @ direction
        expected_transformed_direction = np.array([[0, -1, 0, 0]])
        error = np.linalg.norm(transformed_direction - expected_transformed_direction, ord='fro')
        assert error < 1e-10

        tracker_configuration = TrackerState(10, 20, z, np.pi / 8)
        transformation = self.get_transformation(tracker_configuration)
        direction = np.array([0, 1, 0, 0])
        transformed_direction = transformation @ direction
        expected_transformed_direction = np.array([[-np.sin(np.pi / 8), -np.cos(np.pi / 8), 0, 0]])
        error = np.linalg.norm(transformed_direction - expected_transformed_direction, ord='fro')
        assert error < 1e-10

        self.screws = original_screws
        self.end_effector_at_zero_position = original_end_effector_at_zero_position

if __name__ == '__main__':
    robot_configuration = RobotState.from_robot_parameters(0, 0, 0, 0)
    robot_topology = RobotTopology(l1=10, l2=10, l3=10, h1=30, angle_wide_1=180, angle_wide_2=180, angle_wide_3=180)
    forward_kinematics = RobotForwardKinematics(robot_topology)
    tracker_fordward_kinematics = TrackerForwardKinematics()



