from robotcom.jointdevice import JointDevice
from multiprocessing import Process, Manager
import protocol
import logging
import math
import time

class RobotCommunication():
    '''
        Not using Processing pool as protocol can't be pickled
        Not using threading pool as serial connections are
        somehow lost and there is no noticeable performance improvement (the 'somehow lost' problem might be now fixed)
    '''

    def __init__(self):
        self.linear_joint_0_device = JointDevice('Linear 0', protocol.ActuatorType.LINEAR, '/dev/ttyS5', True, 27, 26, 0, 1000, 51000)
        self.angular_joint_1_device = JointDevice('Angular 1', protocol.ActuatorType.ROTARY, '/dev/ttyS6', True, 27, 26, -425, -18125, 18125)
        self.angular_joint_2_device = JointDevice('Angular 2', protocol.ActuatorType.ROTARY, '/dev/ttyS11', True, 27, 26, -425, -26000, 26000)
        self.angular_joint_3_device = JointDevice('Angular 3', protocol.ActuatorType.ROTARY, '/dev/ttyS10', False, 27, 26, -425, -26000, 26000)

        self.angular_joints = [self.angular_joint_1_device, self.angular_joint_2_device, self.angular_joint_3_device]
        self.linear_joints = [self.linear_joint_0_device]
        self.joints = self.linear_joints + self.angular_joints

        self.driver_steps = 25000
        self.gear_ratio = 20 / 58
        self.linear_ratio = 100 / 20000

        self.logger = logging.getLogger(f'Robot Comunication')
        self.logger.setLevel(logging.DEBUG)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)

    def open(self):
        [joint.open() for joint in self.joints]

    def configure(self):
        configuration_processes = [Process(target=RobotCommunication.configure_joint, args=(joint, self.logger, )) for id, joint in enumerate(self.joints)]
        [configure_process.start() for configure_process in configuration_processes]
        [configure_process.join() for configure_process in configuration_processes]

    @staticmethod
    def configure_joint(joint, logger):
        logger.debug(f'Configuring joint {joint.label}')
        joint.configure_until_finished()
        logger.debug(f'Joint {joint.label} configured')

    def home(self):
        linear_home_process = Process(target=RobotCommunication.home_joint, args=(self.linear_joint_0_device, self.logger, ))
        linear_home_process.start()
        [RobotCommunication.home_joint(angular_joint, self.logger) for id, angular_joint in enumerate(self.angular_joints)]
        linear_home_process.join()

    @staticmethod
    def configure_controller_joint(joint, error_constant, max_microseconds_delay, logger):
        logger.debug(f'Configuring controller joint {joint.label}')
        joint.configure_controller(error_constant, max_microseconds_delay)
        logger.debug(f'Joint {joint.label} contorller configured')

    @staticmethod
    def home_joint(joint, logger):
        logger.debug(f'Homing joint {joint.label}')
        joint.home_until_finished()
        logger.debug(f'Joint {joint.label} homed')

    def move_to_steps(self, target_steps_per_axis):
        move_processes = [Process(target=RobotCommunication.move_joint, args=(joint, target_steps_per_axis[id], self.logger)) for id, joint in enumerate(self.joints)]
        [move_process.start() for move_process in move_processes]
        [move_process.join() for move_process in move_processes]

    @staticmethod
    def move_joint(joint, target_steps, logger):
        logger.debug(f'Moving joint {joint.label} to {target_steps}')
        result = joint.set_target_steps(target_steps)
        logger.debug(f'Joint {joint.label} moved to {result}')

    def close(self):
        close_processes = [Process(target=RobotCommunication.close_joint, args=(joint, self.logger, )) for id, joint in enumerate(self.joints)]
        [close_process.start() for close_process in close_processes]
        [close_process.join() for close_process in close_processes]

    @staticmethod
    def close_joint(joint, logger):
        logger.debug(f'Closing joint {joint.label}')
        joint.close()
        logger.debug(f'Joint {joint.label} closed')

    def wait_until_target_reached(self, target_steps):
        joint_to_current_steps = self.get_steps()
        while joint_to_current_steps != target_steps:
            self.logger.debug(f'Target not reached. Expected {target_steps}. Found {joint_to_current_steps}')
            joint_to_current_steps = self.get_steps()

    def get_steps(self):
        manager = Manager()
        joint_to_current_steps = manager.dict()
        current_steps_processes = [Process(target=RobotCommunication.get_steps_joint, args=(id, joint, joint_to_current_steps, self.logger,)) for id, joint in enumerate(self.joints)]
        [current_steps_process.start() for current_steps_process in current_steps_processes]
        [current_steps_process.join() for current_steps_process in current_steps_processes]
        return [joint_to_current_steps[id] for id, joint in enumerate(self.joints)]

    @staticmethod
    def get_steps_joint(id, joint, joint_to_current_steps, logger):
        logger.debug(f'Getting steps joint {joint.label}')
        joint_to_current_steps[id] = joint.get_steps()
        logger.debug(f'Joint {joint.label} got steps {joint_to_current_steps[id]}')

    def get_target_steps(self):
        manager = Manager()
        joint_to_target_steps = manager.dict()
        target_steps_processes = [Process(target=RobotCommunication.get_target_steps_joint, args=(id, joint, joint_to_target_steps, self.logger,)) for id, joint in enumerate(self.joints)]
        [target_steps_processes.start() for target_steps_processes in target_steps_processes]
        [target_steps_processes.join() for target_steps_processes in target_steps_processes]
        return [joint_to_target_steps[id] for id, joint in enumerate(self.joints)]

    @staticmethod
    def get_target_steps_joint(id, joint, joint_to_target_steps, logger):
        logger.debug(f'Getting target steps joint {joint.label}')
        joint_to_target_steps[id] = joint.get_target_steps()
        logger.debug(f'Joint {joint.label} got target steps {joint_to_target_steps[id]}')

    def angle_to_steps(self, angle):
        if angle > math.pi:
            angle = angle - (2 * math.pi)
        return int((self.driver_steps * angle) / ((2 * math.pi) * self.gear_ratio ))

    def steps_to_angle(self, steps):
        angle = (steps * ((2 * math.pi) * self.gear_ratio )) / self.driver_steps
        if angle < 0:
            angle = (2 * math.pi) + angle
        return angle

    def distance_to_steps(self, distance):
        return int(distance / self.linear_ratio)

    def steps_to_distance(self, steps):
        return self.linear_ratio * steps

    def steps_to_parameters(self, steps):
        linear_parameter = self.steps_to_distance(steps[0])
        angular_parameters = [self.steps_to_angle(step) for index, step in enumerate(steps) if index > 0]
        return [linear_parameter] + angular_parameters

    def parameters_to_steps(self, parameters):
        linear_steps = self.distance_to_steps(parameters[0])
        angular_steps = [self.angle_to_steps(step) for index, step in enumerate(parameters) if index > 0]
        return [linear_steps] + angular_steps

    def move_to_parameters(self, parameters):
        steps = self.parameters_to_steps(parameters)
        self.logger.debug(f'Moving to parameters: {parameters} using steps {steps}')
        self.move_to_steps(steps)
        return steps

    def setup(self):
        self.open()
        self.configure()
        self.home()

if __name__ == '__main__':

    robot_communication = RobotCommunication()
    robot_communication.setup()
    robot_communication.angular_joint_1_device.configure_controller(5000, 5000)
    robot_communication.angular_joint_2_device.configure_controller(5000, 5000)
    robot_communication.angular_joint_3_device.configure_controller(5000, 5000)
    target_parameters = [30, math.pi / 2.0, math.pi / 8.0, math.pi / 16.0 ]
    mult = 1
    for i in range(0, 30):
        new_params = robot_communication.steps_to_parameters(robot_communication.parameters_to_steps(target_parameters))
        move_proceses = robot_communication.move_to_parameters(target_parameters)
        #robot_communication.wait_until_target_reached(target_steps)
        time.sleep(3)
        print(target_parameters, new_params)
        time.sleep(20)
        mult = mult * -1
        target_parameters = [e * mult if idx > 0 else e for idx, e in enumerate(target_parameters)]
    robot_communication.close()
    '''
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
    '''