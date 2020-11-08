from robotcom.jointdevice import JointDevice
from multiprocessing import Process, Manager
import protocol
import logging
import math
import time
from robotcom.robot import RobotCommunication

class SimulationRobotCommunication(RobotCommunication):

    def __init__(self, parameters):
        self.driver_steps = 25000
        self.gear_ratio = 20 / 58
        self.linear_ratio = 100 / 20000

        self.logger = logging.getLogger(f'Simultion Robot Comunication')
        self.logger.setLevel(logging.DEBUG)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)
        self.steps = self.parameters_to_steps(parameters)

    def open(self):
        pass

    def configure(self):
        pass

    def home(self):
        pass

    def move_to_steps(self, target_steps_per_axis):
        self.steps = target_steps_per_axis

    def close(self):
        pass

    def wait_until_target_reached(self, target_steps):
        pass

    def get_steps(self):
        time.sleep(1)
        return self.steps

    def get_target_steps(self):
        time.sleep(1)
        return self.steps

    def move_to_parameters(self, parameters):
        self.steps = self.parameters_to_steps(parameters)

    def setup(self):
        pass

if __name__ == '__main__':
    from robotcontroller.kinematics import RobotTopology
    from robotcontroller.ik import IkSolver
    robot_topology = RobotTopology(l1=142, l2=142, l3=142, h1=30, angle_wide_1=180, angle_wide_2=180, angle_wide_3=180)
    ik_solver = IkSolver(robot_topology)
    robot_communication = RobotCommunication()
    robot_communication.setup()
    samples = 25
    radius = 80
    angle_step = ( (2 * math.pi) / samples)
    loops = 10
    for loop in range(loops):
        for sample in range(samples):
            x_target = (426 - radius) + (radius * math.cos(sample *  angle_step))
            y_target = radius * math.sin(sample * angle_step)
            ik_solutions = ik_solver.compute_ik(dx=1, dy=0, x=x_target, y=y_target)
            if (len(ik_solutions) > 0):
                ik_solution = ik_solutions[0]
                print(ik_solution)
                linear_position = (sample * 5) + 50
                target_position = [linear_position] + ik_solution.angle_solution
                move_proceses = robot_communication.move_to_position(target_position)
                #scara_robot.wait_until_target_reached(target_steps)
            else:
                print('No IK solution')
    ik_controller.close()