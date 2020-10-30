from robotcom.jointdevice import JointDevice
from multiprocessing import Process, Manager
import protocol
import logging

class ScaraRobot():
    '''
        Not using Processing pool as protocol can't be pickled
        Not using threading pool as serial connections are
        somehow lost and there is no noticeable performance improvement
    '''

    def __init__(self):
        self.linear_joint_0_device = JointDevice('Linear 0', protocol.ActuatorType.LINEAR, '/dev/ttyS5', True, 27, 26, 0)
        self.angular_joint_1_device = JointDevice('Angular 1', protocol.ActuatorType.ROTARY, '/dev/ttyS6', True, 27, 26, -425)
        self.angular_joint_2_device = JointDevice('Angular 2', protocol.ActuatorType.ROTARY, '/dev/ttyS11', True, 27, 26, -425)
        self.angular_joint_3_device = JointDevice('Angular 3', protocol.ActuatorType.ROTARY, '/dev/ttyS10', True, 27, 26, -425)
        self.angular_joints = [self.angular_joint_1_device, self.angular_joint_2_device, self.angular_joint_3_device]
        self.linear_joints = [self.linear_joint_0_device]
        self.joints = self.linear_joints + self.angular_joints

        self.logger = logging.getLogger(f'Scara Robot')
        self.logger.setLevel(logging.DEBUG)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)

    def open(self):
        [joint.open() for joint in self.joints]

    def configure(self):
        configuration_processes = [Process(target=ScaraRobot.configure_joint, args=(id, joint, self.logger, )) for id, joint in enumerate(self.joints)]
        [configure_process.start() for configure_process in configuration_processes]
        [configure_process.join() for configure_process in configuration_processes]

    @staticmethod
    def configure_joint(id, joint, logger):
        logger.debug(f'Configuring joint {id}')
        joint.configure_until_finished()
        logger.debug(f'Joint {id} configured')

    def home(self):
        configure_controller_processes = [Process(target=ScaraRobot.configure_controller_joint, args=(id, joint, 5000, 500, self.logger, )) for id, joint in enumerate(self.joints)]
        [configure_controller_process.start() for configure_controller_process in configure_controller_processes]
        [configure_controller_process.join() for configure_controller_process in configure_controller_processes]

        linear_home_process = Process(target=ScaraRobot.home_joint, args=(0, self.linear_joint_0_device, self.logger, ))
        linear_home_process.start()
        [ScaraRobot.home_joint(id + 1, angular_joint, self.logger) for id, angular_joint in enumerate(self.angular_joints)]
        linear_home_process.join()

    @staticmethod
    def configure_controller_joint(id, joint, error_constant, max_microseconds_delay, logger):
        logger.debug(f'Configuring controller joint {id}')
        joint.configure_controller(error_constant, max_microseconds_delay)
        logger.debug(f'Joint {id} contorller configured')

    @staticmethod
    def home_joint(id, joint, logger):
        logger.debug(f'Homing joint {id}')
        joint.home_until_finished()
        logger.debug(f'Joint {id} homed')

    def move(self, target_steps_per_axis):
        move_processes = [Process(target=ScaraRobot.move_joint, args=(id, joint, target_steps_per_axis[id], self.logger)) for id, joint in enumerate(self.joints)]
        [move_process.start() for move_process in move_processes]
        [move_process.join() for move_process in move_processes]

    @staticmethod
    def move_joint(id, joint, target_steps, logger):
        logger.debug(f'Moving joint {id} to {target_steps}')
        result = joint.set_target_steps(target_steps)
        logger.debug(f'Joint {id} moved to {result}')

    def close(self):
        close_processes = [Process(target=ScaraRobot.close_joint, args=(id, joint, self.logger, )) for id, joint in enumerate(self.joints)]
        [close_process.start() for close_process in close_processes]
        [close_process.join() for close_process in close_processes]

    @staticmethod
    def close_joint(id, joint, logger):
        logger.debug(f'Closing joint {id}')
        joint.close()
        logger.debug(f'Joint {id} closed')

    def wait_until_target_reached(self, target_steps):
        joint_to_current_steps = self.get_steps()
        while joint_to_current_steps != target_steps:
            self.logger.debug(f'Target not reached. Expected {target_steps}. Found {joint_to_current_steps}')
            joint_to_current_steps = self.get_steps()

    def get_steps(self):
        manager = Manager()
        joint_to_current_steps = manager.dict()
        current_steps_processes = [Process(target=ScaraRobot.get_steps_joint, args=(id, joint, joint_to_current_steps, self.logger,)) for id, joint in enumerate(self.joints)]
        [current_steps_process.start() for current_steps_process in current_steps_processes]
        [current_steps_process.join() for current_steps_process in current_steps_processes]
        return [joint_to_current_steps[id] for id, joint in enumerate(self.joints)]

    @staticmethod
    def get_steps_joint(id, joint, joint_to_current_steps, logger):
        logger.debug(f'Getting steps joint {id}')
        joint_to_current_steps[id] = joint.get_steps()
        logger.debug(f'Joint {id} got steps {joint_to_current_steps[id]}')

if __name__ == '__main__':
    scara_robot = ScaraRobot()
    scara_robot.open()
    scara_robot.configure()
    scara_robot.angular_joint_1_device.configure_controller(5000, 500)
    scara_robot.angular_joint_2_device.configure_controller(5000, 500)
    scara_robot.angular_joint_3_device.configure_controller(5000, 500)
    scara_robot.home()
    scara_robot.angular_joint_1_device.configure_controller(5000, 5000)
    scara_robot.angular_joint_2_device.configure_controller(5000, 5000)
    scara_robot.angular_joint_3_device.configure_controller(5000, 5000)
    target_steps = [30000, 1000, -1134, -1000]
    mult = 1
    for i in range(0, 30):
        move_proceses = scara_robot.move(target_steps)
        scara_robot.wait_until_target_reached(target_steps)
        mult = mult * -1
        target_steps = [e * mult if idx > 0 else e for idx, e in enumerate(target_steps)]
    scara_robot.close()
