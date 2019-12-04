import numpy as np
import cv2
import glob
import os
import math
import sys 

CAMERA_MATRIX_FILENAME = "camera_matrix.numpy"
DISTORITION_COEFFICIENTS_FILENAME = "distortion_coefficients.numpy"

def to_degrees(radians):
    degrees = ( radians * 180 ) / np.pi
    if(degrees < 0):
        degrees = degrees + 360
    return degrees

def compute_euler_angles_from_rodrigues_vector(rvec):
    rotation_matrix, jacobian = cv2.Rodrigues(rvec)
    #Extract Euler angles from: https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
    angle_x = math.atan2(rotation_matrix[2,1], 
        rotation_matrix[2,2])
    angle_y = math.atan2(-rotation_matrix[2,0], 
        math.sqrt( (rotation_matrix[2,1] ** 2) + (rotation_matrix[2,2] ** 2) ))
    angle_z = math.atan2(rotation_matrix[1,0], rotation_matrix[0,0])
    return to_degrees(angle_x), to_degrees(angle_y), to_degrees(angle_z)

def draw(image, corners, image_points):
    image_points = np.int32(image_points).reshape(-1,2)
    image = cv2.drawContours(image, [image_points[:4]],-1,(0,255,0),-3)
    for i,j in zip(range(4),range(4,8)):
        image = cv2.line(image, tuple(image_points[i]), tuple(image_points[j]),(255),3)
    image = cv2.drawContours(image, [image_points[4:]],-1,(0,0,255),3)
    return image

def get_camera_matrix(image):
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = (np.mgrid[0:7,0:6].T.reshape(-1,2) * 5) - 15
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imagepoints = []
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imagepoints.append(corners2)
        frame = cv2.drawChessboardCorners(image, (7,6), corners2,ret)
        ret, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors = \
            cv2.calibrateCamera(objpoints, imagepoints, gray.shape[::-1], None, None)
        with open(CAMERA_MATRIX_FILENAME, "wb") as camera_matrix_file:
            np.save(camera_matrix_file, camera_matrix)
        with open(DISTORITION_COEFFICIENTS_FILENAME, "wb") as dist_file:
            np.save(dist_file, distortion_coefficients)
        return camera_matrix, distortion_coefficients
    return None, None

def get_position_and_orientation(image, camera_matrix, distortion_coefficients):
    object_points = np.zeros(( 6 * 7, 3 ), np.float32)
    object_points[:,:2] = (np.mgrid [0:7, 0:6 ].T.reshape(-1, 2) * 5) - 15
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    #axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
    #                [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    chessboard_is_found, corners = cv2.findChessboardCorners(gray, (7, 6), None)
    if chessboard_is_found:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1,-1), criteria)
        retval, rotation_vectors, translation_vectors, inliers = \
            cv2.solvePnPRansac(object_points, corners2, camera_matrix, distortion_coefficients)
        #imagepts, jac = cv2.projectPoints(axis, rotation_vectors, translation_vectors, camera_matrix, distortion_coefficients)
        #image = draw(image,corners2,imagepts)
        angle_x, angle_y, angle_z = compute_euler_angles_from_rodrigues_vector(rotation_vectors)
        return angle_x, angle_y, angle_z, translation_vectors
    else:
        return None, None, None, None

def load_camera_matrix():
    camera_matrix = None
    distortion_coefficients = None
    if os.path.exists(CAMERA_MATRIX_FILENAME) and os.path.exists(DISTORITION_COEFFICIENTS_FILENAME):
        with open(CAMERA_MATRIX_FILENAME, "rb") as camera_matrix_file:
            camera_matrix = np.load(camera_matrix_file)
        with open(DISTORITION_COEFFICIENTS_FILENAME, "rb") as dist_file:
            distortion_coefficients = np.load(dist_file)
    return camera_matrix, distortion_coefficients

video_capture = cv2.VideoCapture(4)
camera_matrix, distortion_coefficients = None, None
while video_capture.isOpened():
    ret, image = video_capture.read()
    if camera_matrix is None or distortion_coefficients is None:
         camera_matrix, distortion_coefficients = get_camera_matrix(image)
    angle_x, angle_y, angle_z, translation = get_position_and_orientation(image, camera_matrix, distortion_coefficients)
    print(angle_x, angle_y, angle_z, translation)
    cv2.imshow('image', image)
    key = cv2.waitKey(0) & 0xff
    if key == 'q':
        sys.exit(1)

cv2.destroyAllWindows()
   