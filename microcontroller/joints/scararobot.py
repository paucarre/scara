from jointdevice import JointDevice
from multiprocessing import Process
import protocol

class ScaraRobot():

    def __init__(self):
        self.linear_joint_0_device = JointDevice(protocol.ActuatorType.LINEAR, '/dev/ttyS5', False, 27, 26, 0)
        self.angular_joint_1_device = JointDevice(protocol.ActuatorType.ROTARY, '/dev/ttyS6', True, 27, 26, -425)
        self.angular_joint_2_device = JointDevice(protocol.ActuatorType.ROTARY, '/dev/ttyS11', True, 27, 26, -425)
        self.angular_joint_3_device = JointDevice(protocol.ActuatorType.ROTARY, '/dev/ttyS10', True, 27, 26, -425)
        self.angular_joints = [self.angular_joint_1_device, self.angular_joint_2_device, self.angular_joint_3_device]
        self.linear_joints = [self.linear_joint_0_device]
        self.joints = self.linear_joints + self.angular_joints

    def open(self):
        [joint.open() for joint in self.joints]

    def configure(self):
        configuration_processes = [Process(target=ScaraRobot.configure_joint, args=(id, joint,)) for id, joint in enumerate(self.joints)]
        [configure_process.start() for configure_process in configuration_processes]
        [configure_process.join() for configure_process in configuration_processes]

    def home(self):
        linear_home_process = Process(target=ScaraRobot.home_joint, args=(0, self.linear_joint_0_device,))
        linear_home_process.start()
        [ScaraRobot.home_joint(id + 1, angular_joint) for id, angular_joint in enumerate(self.angular_joints)]
        linear_home_process.join()

    @staticmethod
    def configure_joint(id, joint):
        print(f'Configuring joint {id}')
        joint.configure_until_finished()
        print(f'Joint {id} configured')

    @staticmethod
    def home_joint(id, joint):
        print(f'Homing joint {id}')
        joint.home_until_finished()
        print(f'Joint {id} homed')

if __name__ == '__main__':
    scara_robot = ScaraRobot()
    scara_robot.open()
    scara_robot.configure()
    scara_robot.home()
