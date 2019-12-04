from imutils.video import VideoStream
import imagezmq
import numpy as np
import cv2
import os
from kinematics import TrackerState
from PIL import Image
import math

CAMERA_MATRIX_FILENAME = "../camera/camera_matrix.numpy"
DISTORITION_COEFFICIENTS_FILENAME = "../camera/distortion_coefficients.numpy"


#TODO: remove, not used and unlikely it will ever be
def to_degrees(radians):
    degrees = ( radians * 180 ) / np.pi
    if(degrees < 0):
        degrees = degrees + 360
    return degrees


def compute_euler_angles_from_rodrigues_vector(rvec):
    #TODO: anle_z is correct, but angle_x and angle_y **might** need to be swapped
    # this needs to be checked (if ever used and needed!)
    rotation_matrix, jacobian = cv2.Rodrigues(rvec)
    #Extract Euler angles from: https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
    angle_x = math.atan2(rotation_matrix[2,1], 
        rotation_matrix[2,2])
    angle_z = math.atan2(-rotation_matrix[2,0], 
        math.sqrt( (rotation_matrix[2,1] ** 2) + (rotation_matrix[2,2] ** 2) ))
    angle_y = math.atan2(rotation_matrix[1,0], rotation_matrix[0,0])
    return angle_x, angle_y, angle_z

def draw(image, corners, image_points):
    image_points = np.int32(image_points).reshape(-1,2)
    image = cv2.drawContours(image, [image_points[:4]],-1,(0,255,0),-3)
    for i,j in zip(range(4),range(4,8)):
        image = cv2.line(image, tuple(image_points[i]), tuple(image_points[j]),(255),3)
    image = cv2.drawContours(image, [image_points[4:]],-1,(0,0,255),3)
    return image

def get_position_and_orientation(image, camera_matrix, distortion_coefficients):
    object_points = np.zeros(( 6 * 7, 3 ), np.float32)
    object_points[:,:2] = (np.mgrid [0:7, 0:6 ].T.reshape(-1, 2) * 5) - 15
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    chessboard_is_found, corners = cv2.findChessboardCorners(image, (7, 6),cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if chessboard_is_found:
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.CALIB_CB_FILTER_QUADS | \
            cv2.CALIB_FIX_S1_S2_S3_S4 | \
            cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 \
            | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6 | cv2.CALIB_FIX_FOCAL_LENGTH, 100, 0.1)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
        retval, rotation_vectors, translation_vectors, inliers = \
            cv2.solvePnPRansac(object_points, corners2, camera_matrix, distortion_coefficients)
        axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
            [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
        image_points, jac = cv2.projectPoints(axis, rotation_vectors, translation_vectors, 
            camera_matrix, distortion_coefficients)
        image = draw(image, corners2, image_points)
        angle_x, angle_y, angle_z = compute_euler_angles_from_rodrigues_vector(rotation_vectors)
        tracker_configuration = TrackerState(-translation_vectors[1][0], -translation_vectors[0][0], 
            translation_vectors[2][0], angle_z)
        return image, tracker_configuration
    else:
        return None, None

def load_camera_matrix():
    camera_matrix = None
    distortion_coefficients = None
    if os.path.exists(CAMERA_MATRIX_FILENAME) and os.path.exists(DISTORITION_COEFFICIENTS_FILENAME):
        with open(CAMERA_MATRIX_FILENAME, "rb") as camera_matrix_file:
            camera_matrix = np.load(camera_matrix_file)
        with open(DISTORITION_COEFFICIENTS_FILENAME, "rb") as dist_file:
            distortion_coefficients = np.load(dist_file)
    return camera_matrix, distortion_coefficients