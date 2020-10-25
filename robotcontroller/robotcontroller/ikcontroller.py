from kinematics import RobotTopology, RobotState, RobotForwardKinematics, TrackerForwardKinematics, TrackerState
from ik import IkSolver
from robotcom.scararobot import ScaraRobot

class IkController():

    def __init__(self, robot_topology):
        self.robot_topology = robot_topology
        self.robot_forward_kinamatics = RobotForwardKinematics(robot_topology)
        self.ik_solver = IkSolver(robot_topology)
        self.driver_steps = 25000
        self.gear_ratio = 20 / 58

    def steps_to_angle(self, angle):
        return int(self.driver_steps * angle / (360.0 * self.gear_ratio ))

    def get_angles(self, x, y, z, dx, dy):
        ik_solutions = self.ik_solver.compute_ik(dx=dx, dy=dy, x=x, y=y)
        return ik_solutions

    def get_steps(self, ik_solution):
        steps = [self.steps_to_angle(angle) for angle in ik_solution.angle_solution]
        return steps


if __name__ == '__main__':
    robot_topology = RobotTopology(l1=10, l2=10, l3=10, h1=30, angle_wide_1=180, angle_wide_2=180, angle_wide_3=180)
    ik_controller = IkController(robot_topology)
    ik_solutions = ik_controller.get_angles(0, 10, 0, 0, 1)
    if (len(ik_solutions) > 0):
        ik_solution = ik_solutions[0]
        print(ik_solution)
        steps_solution = ik_controller.get_steps(ik_solution)
        scara_robot = ScaraRobot()
        scara_robot.open()
        scara_robot.configure()
        scara_robot.home()
        target_steps = [0] + steps_solution
        move_proceses = scara_robot.move(target_steps)
        scara_robot.wait_until_target_reached(target_steps)
        scara_robot.close()
    else:
        print('No IK solution')