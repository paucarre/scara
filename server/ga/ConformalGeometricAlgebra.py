from clifford import *
import random
import math

class ConformalGeometricAlgebra(object):

    def __init__(self, resolution=1e-15):
        self.layout, self.blades = Cl(4,1)
        pretty()
        self.resolution = resolution
        eps(self.resolution)
        self.e1, self.e2, self.e3, self.e_hat, self.e = [self.blades['e%i'%k] for k in range(1, 6)]
        self.e_origin = 0.5 ^ (self.e - self.e_hat)
        self.e_inf = self.e_hat + self.e
        self.minkowski_plane = self.e_inf ^ self.e_origin

    def angle_from_rotor(self, rotor):
        cos_half_angle = rotor.lc(1)
        half_angle = math.acos(cos_half_angle)
        angle = half_angle * 2.0
        if(float((rotor - cos_half_angle) | rotor.lc((self.e1^self.e2)+(self.e1^self.e3)+(self.e2^self.e3))) < 0.0):
            angle = -angle
        return angle

    def to_rotor(self, first_vector, second_vector):
        second_vector  = self.normalize_vector(second_vector)
        first_vector   = self.normalize_vector(first_vector)
        rotation_plane = self.normalize_vector(second_vector ^ first_vector)
        angle = self.angle(first_vector, second_vector)
        return self.rotor(rotation_plane, angle)

    def to_point(self, vector):
        return self.homogeneous_point(vector + (0.5 ^ ( (vector**2) * self.e_inf ) ) + self.e_origin)

    def homogeneous_point(self, point):
        if(abs(point | self.e_inf) > self.resolution):
            return point * ( -point | self.e_inf ).normalInv()
        else:
            # zero point, non-invertible
            return self.point(0.0, 0.0, 0.0)

    def to_vector(self, point):
        return ( self.homogeneous_point(point) ^ self.minkowski_plane ) * self.minkowski_plane

    def rotor(self, bivector, angle):
        return math.cos(angle / 2.0) + (math.sin(angle / 2.0) ^ bivector)

    def translator(self, vector):
        return 1.0 + ( 0.5 ^ (self.e_inf * vector) )

    def project(self, point_pair):
        beta = math.sqrt(abs(point_pair * point_pair))
        point_pair = (1.0 / beta) * point_pair
        projector = 0.5 ^ (1.0 + point_pair)
        first_point = projector * (point_pair | self.e_inf)
        second_point = - ~projector * (point_pair | self.e_inf)
        return first_point, second_point

    def act(self, element, transformation):
        return transformation * element * (~transformation)

    def acts(self, element, transformations):
        for transformation in transformations:
            element = self.act(element, transformation)
        return element

    def vector(self, x, y, z):
        return (x^self.e1) + (y^self.e2) + (z^self.e3)

    def point(self, x, y, z):
        return self.to_point(self.vector(x, y, z))

    def line(self, first_point, second_point):
        return self.homogeneous_point(first_point) ^ self.homogeneous_point(second_point) ^ self.e_inf

    def norm(self, vector):
        return math.sqrt(abs(vector * ~vector))

    def normalize_vector(self, vector):
        norm2 = self.norm(vector)
        if norm2 > self.resolution:
            return vector / norm2
        else:
            return vector

    def direction(self, source_position, destination_position):
        return self.normalize_vector(self.to_vector(destination_position) - self.to_vector(source_position))

    def angle(self, first_vector, second_vector):
        first_vector_normalized = self.normalize_vector(first_vector)
        second_vector_normalized = self.normalize_vector(second_vector)
        cos_angle = float(first_vector_normalized | second_vector_normalized)
        if(cos_angle < -1.0):
            cos_angle = -1.0
        if(cos_angle > 1.0):
            cos_angle = 1.0
        angle = math.acos(cos_angle)
        #if(angle > math.pi):
        #    angle = (2.0 * math.pi) - angle
        return angle

    def distance(self, origin_point, destination_point):
        distance = destination_point - origin_point
        return math.sqrt(abs(distance * ~distance))

    def plane(self, point_1, point_2, point_3):
        return point_1 ^ point_2 ^ point_3 ^ self.e_inf

    def sphere(self, center, radius):
        sphere_point_1_translation = self.translator(self.vector(radius, 0.0, 0.0))
        sphere_point_2_translation = self.translator(self.vector(0.0, radius, 0.0))
        sphere_point_3_translation = self.translator(self.vector(0.0, 0.0, radius))
        sphere_point_4_translation = self.translator(self.vector(-radius, 0.0, 0.0))
        sphere = self.act(center, sphere_point_1_translation) ^ \
                 self.act(center, sphere_point_2_translation) ^ \
                 self.act(center, sphere_point_3_translation) ^ \
                 self.act(center, sphere_point_4_translation)
        return sphere
