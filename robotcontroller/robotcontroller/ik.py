from robotcontroller.kinematics import RobotForwardKinematics, RobotTopology

import numpy as np
import math
from robotcontroller.ga.ConformalGeometricAlgebra import ConformalGeometricAlgebra
import traceback
import sys

class IkSolution():

    def __init__(self, angle_solution, cartesian_solution, solution_type):
        self.angle_solution = angle_solution
        self.cartesian_solution = cartesian_solution
        self.solution_type = solution_type

    def __repr__(self):
       return f'Angle: {self.angle_solution} | Cartesian: {self.cartesian_solution} | Type: {self.solution_type}'

class IkSolver():

    def __init__(self, robot_topology):
        self.robot_topology = robot_topology
        self.cga = ConformalGeometricAlgebra()
        self.resolution = 1e-3
        self.plane = self.cga.plane(self.cga.e_origin, self.cga.to_point(self.cga.e1), self.cga.to_point(self.cga.e2))
        self.test()

    @staticmethod
    def angle_from_vector(dx, dy):
        delta_quadrant = 0
        if(dx > 0.0 and dy >= 0.0):
            # first quadrant
            delta_quadrant = 0
        elif(dx <= 0.0 and dy > 0.0):
            # second quadrant
            dx, dy = dy, dx
            delta_quadrant = math.pi / 2.0
        elif(dx < 0.0 and dy <= 0.0):
            # third quadrant
            delta_quadrant = math.pi
        elif(dx >= 0.0 and dy <= 0.0):
            # forth quadrant
            dx, dy = dy, dx
            delta_quadrant = math.pi + (math.pi / 2.0)
        alpha = 0.0
        if(abs(dx) > 1e-30):
            alpha = math.atan(abs(dy / dx))
        alpha = alpha + delta_quadrant
        return alpha

    @staticmethod
    def to_grads(radians):
        return radians * 360 / (2 * math.pi)

    @staticmethod
    def to_rads(grads):
        return (grads * math.pi) / 180.0

    @staticmethod
    def normalize_rads(rads):
        while(rads > 2.0 * np.pi):
            rads = rads - (np.pi * 2.0)
        while(rads < 0.0):
            rads = rads + (2.0 * np.pi)
        return rads

    @staticmethod
    def get_solutions(x0, y0, x1, y1, x2, y2, x3, y3, solution_type):
        solution_cartesian = [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
        angle_1 = IkSolver.angle_from_vector(x1 - x0, y1 - y0)
        angle_2 = IkSolver.angle_from_vector(x2 - x1, y2 - y1)
        angle_3 = IkSolver.angle_from_vector(x3 - x2, y3 - y2)

        '''
        # make angles relatives to one another
        angle_1_diff = angle_1
        angle_2_diff = angle_2 - angle_1
        angle_3_diff = angle_3 - angle_2
        angle_1_diff = IkSolver.normalize_rads(angle_1)
        angle_2_diff = IkSolver.normalize_rads(angle_2_diff)
        angle_3_diff = IkSolver.normalize_rads(angle_3_diff)
        '''

        solution_angles = [angle_1, angle_2, angle_3]

        return IkSolution(solution_angles, solution_cartesian, solution_type)


    @staticmethod
    def angle_is_within_range(angle, angle_wide):
        if (angle > np.pi):
            angle = angle - (2.0 * np.pi)
        if (angle < 0.0):
            return angle > - angle_wide / 2.0
        else:
            return angle < angle_wide / 2.0

    def point_to_x_y(self, point):
        vector = self.cga.to_vector(point)
        x = float(vector.lc(self.cga.e1))
        y = float(vector.lc(self.cga.e2))
        return x, y

    @staticmethod
    def normalize(x, y):
        norm = math.sqrt( (x ** 2) + (y ** 2) )
        x = x / norm
        y = y / norm
        return x, y

    def points_to_solutions(self, point_0, point_1, point_2, point_3, solution_type):
        x0, y0 = self.point_to_x_y(point_0)
        x1, y1 = self.point_to_x_y(point_1)
        x2, y2 = self.point_to_x_y(point_2)
        x3, y3 = self.point_to_x_y(point_3)
        return IkSolver.get_solutions(x0, y0, x1, y1, x2, y2, x3, y3, solution_type)

    def compute_ik_unreacheable(self, x, y):
        dx, dy = IkSolver.normalize(x, y)
        x = dx * (self.robot_topology.l1 + self.robot_topology.l2 + self.robot_topology.l3)
        y = dy * (self.robot_topology.l1 + self.robot_topology.l2 + self.robot_topology.l3)
        solutions = self.compute_ik_position_orientation(dx, dy, x, y, 'UNREACHEABLE')
        if len(solutions) > 0:
            return [solutions[0]]
        else:
            return []

    def compute_ik_position_orientation(self, dx, dy, x, y, solution_type='POS_ORIENT'):
        try:
            dx, dy = IkSolver.normalize(dx, dy)

            point_0 = self.cga.e_origin
            point_3 = self.cga.to_point( ( x ^ self.cga.e1 ) + ( y ^ self.cga.e2 ) )
            point_2 = self.cga.to_point( \
                ( (x - (dx * self.robot_topology.l3) ) ^ self.cga.e1) + \
                ( (y - (dy * self.robot_topology.l3) ) ^ self.cga.e2) )

            p2_centered_sphere = self.cga.sphere(point_2, self.robot_topology.l2)
            p0_centered_sphere = self.cga.sphere(point_0, self.robot_topology.l1)
            p1_dual_point = p0_centered_sphere.meet(p2_centered_sphere).meet(self.plane)
            p1_first, p1_second = self.cga.project(p1_dual_point)
            if abs(self.cga.norm(self.cga.to_vector(p1_first)) - self.robot_topology.l1) < self.resolution and \
                abs(self.cga.norm(self.cga.to_vector(p1_first)) - self.robot_topology.l1) < self.resolution:
                first_solution  = self.points_to_solutions(point_0, p1_first, point_2, point_3, solution_type)
                second_solution = self.points_to_solutions(point_0, p1_second, point_2, point_3, solution_type)
                return [first_solution, second_solution]
            else:
                return []
        except:
            print(traceback.format_exc(), file=sys.stderr)
            return []

    def compute_ik_position(self, x, y):
        return self.compute_ik_position_orientation(x, y, x, y, 'ONLY_POSITION')

    def compute_ik(self, dx, dy, x, y):
        positon_angle_solutions = self.compute_ik_position_orientation(dx, dy, x, y)
        if len(positon_angle_solutions) == 0:
            position_solutions = self.compute_ik_position(x, y)
            if len(position_solutions) == 0:
                return self.compute_ik_unreacheable(x, y)
            else:
                return position_solutions
        else:
            return positon_angle_solutions

    def compute_constrained_ik(self, dx, dy, x, y):
        solutions = self.compute_ik(dx=dx, dy=dy, x=x, y=y)
        # check if any angle is outside the allowed angle
        constrained_solutions = []
        for solution in solutions:
            angle_solution = solution.angle_solution
            if IkSolver.angle_is_within_range(angle_solution[0] - (np.pi / 2.0), IkSolver.to_rads(self.robot_topology.angle_wide_1)) and \
                IkSolver.angle_is_within_range(angle_solution[1], IkSolver.to_rads(self.robot_topology.angle_wide_2)) and \
                IkSolver.angle_is_within_range(angle_solution[2], IkSolver.to_rads(self.robot_topology.angle_wide_3)):
                constrained_solutions.append(solution)
        return constrained_solutions

    @staticmethod
    def angle_difference(angle_1, angle_2):
        return min((2 * np.pi) - abs(angle_1 - angle_2), abs(angle_1 - angle_2))

    def test(self):
        l1_original = self.robot_topology.l1
        l2_original = self.robot_topology.l2
        l3_original = self.robot_topology.l3

        angle_wide_1_original = self.robot_topology.angle_wide_1
        angle_wide_2_original = self.robot_topology.angle_wide_2
        angle_wide_3_original = self.robot_topology.angle_wide_3

        self.robot_topology.l1=10
        self.robot_topology.l2=10
        self.robot_topology.l3=10
        solutions = self.compute_ik(dx=1, dy=1, x=0, y=10)
        solution_0 = solutions[0].angle_solution
        '''
          /
         /
        /
        |
        |
        |
        |  /
        | /
        |/
        '''
        assert abs(solution_0[0] - IkSolver.to_rads(135.0 + 90)) < 0.1
        assert abs(solution_0[1] - IkSolver.to_rads(90)) < 0.1
        assert abs(solution_0[2] - IkSolver.to_rads(45.0)) < 0.1

        solution_1 = solutions[1].angle_solution
        '''
           /|
          / |
         /  |
            |
            |
            |
            |
            |
        '''
        assert abs(solution_1[0] - IkSolver.to_rads(90)) < 0.1
        assert abs(solution_1[1] - IkSolver.to_rads(225.0)) < 0.1
        assert abs(solution_1[2] - IkSolver.to_rads(45)) < 0.1

        assert IkSolver.to_grads(IkSolver.angle_from_vector( 1.0,  1.0)) == 45.0
        assert IkSolver.to_grads(IkSolver.angle_from_vector(-1.0,  1.0)) == 45.0 + 90
        assert IkSolver.to_grads(IkSolver.angle_from_vector(-1.0, -1.0)) == 45.0 + 180
        assert IkSolver.to_grads(IkSolver.angle_from_vector( 1.0, -1.0)) == 45.0 + 270

        assert IkSolver.to_grads(IkSolver.angle_from_vector( 2.0,  1.0)) == 26.56505117707799
        assert IkSolver.to_grads(IkSolver.angle_from_vector(-1.0,  2.0)) == 26.56505117707799 + 90
        assert IkSolver.to_grads(IkSolver.angle_from_vector(-2.0, -1.0)) == 26.56505117707799 + 180
        assert IkSolver.to_grads(IkSolver.angle_from_vector( 1.0, -2.0)) == 26.56505117707799 + 270

        assert IkSolver.to_grads(IkSolver.angle_from_vector( 1.0,  0.0)) == 0.0
        assert IkSolver.to_grads(IkSolver.angle_from_vector( 0.0,  1.0)) == 90.0
        assert IkSolver.to_grads(IkSolver.angle_from_vector(-1.0,  0.0)) == 180.0
        assert IkSolver.to_grads(IkSolver.angle_from_vector( 0.0, -1.0)) == 270.0

        self.robot_topology.angle_wide_1 = 280
        self.robot_topology.angle_wide_2 = 280
        self.robot_topology.angle_wide_3 = 280
        solutions = self.compute_constrained_ik(dx=1, dy=1, x=0, y=25)
        assert len(solutions) == 2
        self.robot_topology.angle_wide_1 = 210
        self.robot_topology.angle_wide_2 = 210
        self.robot_topology.angle_wide_3 = 210
        solutions = self.compute_constrained_ik(dx=1, dy=1, x=0, y=25)
        assert len(solutions) == 1
        self.robot_topology.angle_wide_1 = 100
        self.robot_topology.angle_wide_2 = 100
        self.robot_topology.angle_wide_3 = 100
        solutions = self.compute_constrained_ik(dx=1, dy=1, x=0, y=25)
        assert len(solutions) == 0

        # unreacheable, solves wrong orientation yet raching target
        self.robot_topology.l1=10
        self.robot_topology.l2=10
        self.robot_topology.l3=10
        solutions = self.compute_ik(dx=1, dy=0, x=0, y=20)
        angle_solution = solutions[0].angle_solution
        assert abs(angle_solution[0] - IkSolver.to_rads(60.0 + 90)) < 0.1
        assert abs(angle_solution[1] - IkSolver.to_rads(30)) < 0.1
        assert abs(angle_solution[2] - IkSolver.to_rads(90.0)) < 0.1

        # unreacheable, solves straight line
        self.robot_topology.l1=10
        self.robot_topology.l2=10
        self.robot_topology.l3=10
        self.robot_topology.angle_wide_1 = 280
        self.robot_topology.angle_wide_2 = 280
        self.robot_topology.angle_wide_3 = 280
        solutions = self.compute_constrained_ik(dx=1, dy=0, x=-40, y=40)
        assert len(solutions) == 1
        angle_solution = solutions[0].angle_solution
        assert abs( IkSolver.angle_difference( angle_solution[0], IkSolver.to_rads(45.0 + 90)) ) < 0.1
        assert abs( IkSolver.angle_difference( angle_solution[1], IkSolver.to_rads(135.0)) ) < 0.1
        assert abs( IkSolver.angle_difference( angle_solution[2], IkSolver.to_rads(135.0)) ) < 0.1

        self.robot_topology.angle_wide_1 = angle_wide_1_original
        self.robot_topology.angle_wide_2 = angle_wide_2_original
        self.robot_topology.angle_wide_3 = angle_wide_3_original

        self.robot_topology.l1 = l1_original
        self.robot_topology.l2 = l2_original
        self.robot_topology.l3 = l3_original


if __name__ == '__main__':
    robot_topology = RobotTopology(l1=10, l2=10, l3=10, h1=30, angle_wide_1=180, angle_wide_2=180, angle_wide_3=180)
    ik_solver = IkSolver(robot_topology)
    solutions = ik_solver.compute_constrained_ik(dx=1, dy=1, x=0, y=25)



