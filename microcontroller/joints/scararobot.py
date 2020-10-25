from jointdevice import JointDevice
from multiprocessing import Process, Manager
import protocol

class ScaraRobot():
    '''
        Not using Processing pool as protocol can't be pickled
        Not using threading pool as serial connections are
        somehow lost and there is no noticeable performance improvement
    '''

    def __init__(self):
        self.linear_joint_0_device = JointDevice(protocol.ActuatorType.LINEAR, '/dev/ttyS5', False, 27, 26, 0)
        self.angular_joint_1_device = JointDevice(protocol.ActuatorType.ROTARY, '/dev/ttyS6', True, 27, 26, -425)
        self.angular_joint_2_device = JointDevice(protocol.ActuatorType.ROTARY, '/dev/ttyS11', True, 27, 26, -425)
        self.angular_joint_3_device = JointDevice(protocol.ActuatorType.ROTARY, '/dev/ttyS10', True, 27, 26, -425)
        self.angular_joints = [self.angular_joint_1_device, self.angular_joint_2_device, self.angular_joint_3_device]
        self.linear_joints = [self.linear_joint_0_device]
        self.joints = self.linear_joints + self.angular_joints
        self.thread_pool = ThreadPool(processes=len(self.joints))

    def open(self):
        [joint.open() for joint in self.joints]

    def configure(self):
        configuration_processes = [Process(target=ScaraRobot.configure_joint, args=(id, joint,)) for id, joint in enumerate(self.joints)]
        [configure_process.start() for configure_process in configuration_processes]
        [configure_process.join() for configure_process in configuration_processes]

    @staticmethod
    def configure_joint(id, joint):
        print(f'Configuring joint {id}')
        joint.configure_until_finished()
        print(f'Joint {id} configured')

    def home(self):
        linear_home_process = Process(target=ScaraRobot.home_joint, args=(0, self.linear_joint_0_device,))
        linear_home_process.start()
        [ScaraRobot.home_joint(id + 1, angular_joint) for id, angular_joint in enumerate(self.angular_joints)]
        linear_home_process.join()

    @staticmethod
    def home_joint(id, joint):
        print(f'Homing joint {id}')
        joint.home_until_finished()
        print(f'Joint {id} homed')

    def move(self, target_steps_per_axis):
        move_processes = [Process(target=ScaraRobot.move_joint, args=(id, joint, target_steps_per_axis[id],)) for id, joint in enumerate(self.joints)]
        [move_process.start() for move_process in move_processes]
        [move_process.join() for move_process in move_processes]

    @staticmethod
    def move_joint(id, joint, target_steps):
        print(f'Moving joint {id}')
        result = joint.set_target_steps(target_steps)
        print(f'Joint {id} moved to {result}')

    def close(self):
        close_processes = [Process(target=ScaraRobot.close_joint, args=(id, joint,)) for id, joint in enumerate(self.joints)]
        [close_process.start() for close_process in close_processes]
        [close_process.join() for close_process in close_processes]
        self.pool.close()

    @staticmethod
    def close_joint(id, joint):
        print(f'Closing joint {id}')
        joint.close()
        print(f'Joint {id} closed')

    def wait_until_target_reached(self, target_steps):
        joint_to_current_steps = self.get_steps()
        while joint_to_current_steps != target_steps:
            print(f'Target not reached. Expected {target_steps}. Found {joint_to_current_steps}')
            joint_to_current_steps = self.get_steps()

    def get_steps(self):
        manager = Manager()
        joint_to_current_steps = manager.dict()
        current_steps_processes = [Process(target=ScaraRobot.get_steps_joint, args=(id, joint, joint_to_current_steps, )) for id, joint in enumerate(self.joints)]
        [current_steps_process.start() for current_steps_process in current_steps_processes]
        [current_steps_process.join() for current_steps_process in current_steps_processes]
        return [joint_to_current_steps[id] for id, joint in enumerate(self.joints)]


    @staticmethod
    def get_steps_joint(id, joint, joint_to_current_steps):
        print(f'Getting steps joint {id}')
        joint_to_current_steps[id] = joint.get_steps()
        print(f'Joint {id} got steps {joint_to_current_steps[id]}')

if __name__ == '__main__':
    scara_robot = ScaraRobot()
    scara_robot.open()
    scara_robot.configure()
    scara_robot.home()
    target_steps = [1000, 10000, -10000, -10000]
    mult = 1
    for i in range(0, 50):
        move_proceses = scara_robot.move(target_steps)
        scara_robot.wait_until_target_reached(target_steps)
        mult = mult * -1
        target_steps = [e * mult for e in target_steps]
    scara_robot.close()
