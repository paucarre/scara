from kinematics import RobotTopology, RobotState, RobotForwardKinematics, TrackerForwardKinematics, TrackerState
from ik import IkSolver
from robotcom.scararobot import ScaraRobot
import math
import time
import logging

class IkController():

    def __init__(self, robot_topology):
        self.robot_topology = robot_topology
        self.robot_forward_kinamatics = RobotForwardKinematics(robot_topology)
        self.ik_solver = IkSolver(robot_topology)
        self.driver_steps = 25000
        self.gear_ratio = 20 / 58
        self.linear_ratio = 100 / 20000
        self.scara_robot = ScaraRobot()

        self.logger = logging.getLogger(f'IK Controller')
        self.logger.setLevel(logging.DEBUG)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)


    def angle_to_steps(self, angle):
        if angle > math.pi:
            angle = angle - (2 * math.pi)
        return int(self.driver_steps * angle / ((2 * math.pi) * self.gear_ratio ))

    def distance_to_steps(self, distance):
        return int(distance / self.linear_ratio)

    def get_angles(self, x, y, z, dx, dy):
        ik_solutions = self.ik_solver.compute_ik(dx=dx, dy=dy, x=x, y=y)
        return ik_solutions

    def get_steps(self, ik_solution):
        steps = [self.angle_to_steps(angle) for angle in ik_solution.angle_solution]
        return steps

    def move(self, positions):
        linear_position = self.distance_to_steps(positions[0])
        angular_positions = [self.angle_to_steps(angle) for index, angle in enumerate(positions) if index > 0]
        steps = [linear_position] + angular_positions
        self.logger.debug(f'Moving to positions: {positions} using steps {steps}')
        self.scara_robot.move(steps)

    def setup(self):
        self.scara_robot.open()
        self.scara_robot.configure()
        self.scara_robot.home()

    def shutdown(self):
        self.scara_robot.close()

if __name__ == '__main__':
    robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180, angle_wide_3=180)
    ik_controller = IkController(robot_topology)
    ik_controller.setup()
    samples = 25
    radius = 80
    angle_step = ( (2 * math.pi) / samples)
    loops = 10
    for loop in range(loops):
        for sample in range(samples):
            x_target = (426 - radius) + (radius * math.cos(sample *  angle_step))
            y_target = radius * math.sin(sample * angle_step)
            ik_solutions = ik_controller.get_angles(x_target, y_target, 0, 1, 0)
            if (len(ik_solutions) > 0):
                ik_solution = ik_solutions[0]
                print(ik_solution)
                linear_steps = (sample * 5) + 50
                target_steps = [linear_steps] + ik_solution.angle_solution
                move_proceses = ik_controller.move(target_steps)
                #scara_robot.wait_until_target_reached(target_steps)
            else:
                print('No IK solution')
    ik_controller.shutdown()