import cv2
import numpy as np
import math
import os 

class SquarePoints():

    def __init__(self, top_left, top_right, bottom_left, bottom_right):
        self.top_left = top_left
        self.top_right = top_right
        self.bottom_left = bottom_left
        self.bottom_right = bottom_right
    
    def points(self):
        return np.stack([self.top_left, self.top_right, self.bottom_left, self.bottom_right], axis=0)

    def correspondences(self):
        return np.array([[-10.0, -10.0, 1], [10.0, -10.0, 1], [-10.0, 10.0, 1], [10.0, 10.0, 1]])

    def __repr__(self):
        return str(self.__dict__)

class TrackerOrientation():

    def __init__(self, inverse_camera_matrix):
        self.inverse_camera_matrix = inverse_camera_matrix
        self.last_square_points = None
    
    @staticmethod
    def point_distance(point_a, point_b):
        return math.sqrt (  ( (point_a[0] - point_b[0]) ** 2 ) + ( (point_a[1] - point_b[1]) ** 2 ) )

    @staticmethod
    def get_point_index_to_cluster_id(points, radius=2.0):
        sorted_indices = np.argsort(points[:,0])
        cluster_id = 0
        point_index_to_cluster_id = {}
        all_points_have_clusters = False
        sorted_idx_a = 0
        while(not all_points_have_clusters):
            point_a_idx = sorted_indices[sorted_idx_a]
            point_a = points[point_a_idx]
            sorted_idx_b = sorted_idx_a + 1
            in_cluster_not_fount_attempts = 10
            while(in_cluster_not_fount_attempts > 0 and sorted_idx_b < points.shape[0]):
                point_b_idx = sorted_indices[sorted_idx_b]
                point_b = points[point_b_idx]
                distance = abs(point_a[0] - point_b[0]) + abs(point_a[1] - point_b[1])
                if distance < radius:
                    if point_a_idx not in point_index_to_cluster_id:
                        point_index_to_cluster_id[point_a_idx] = cluster_id
                        cluster_id += 1
                    point_index_to_cluster_id[point_b_idx] = point_index_to_cluster_id[point_a_idx]
                    in_cluster_not_fount_attempts = 10
                else:
                    in_cluster_not_fount_attempts -= 1
                sorted_idx_b += 1
            sorted_idx_a += 1
            all_points_have_clusters = sorted_idx_a == points.shape[0] - 1
        return point_index_to_cluster_id
    
    @staticmethod
    def get_point_indices(point_index_to_cluster_id, points_in_cluster):
        cluster_id_to_point_indices = {}
        for point_index, cluster_id in point_index_to_cluster_id.items():
            if cluster_id not in cluster_id_to_point_indices:
                cluster_id_to_point_indices[cluster_id] = []
            cluster_id_to_point_indices[cluster_id] += [point_index]
        return [indices for cluster_id, indices in cluster_id_to_point_indices.items() if len(indices) >= points_in_cluster]

    @staticmethod
    def get_mean_points(point_indices, points):
        mean_points = []
        for point_indices in point_indices:
            points_in_cluster = [points[point_index] for point_index in point_indices]
            point = np.array(points_in_cluster).mean(0)
            mean_points.append([point[1], point[0], 1])
        return mean_points

    @staticmethod
    def get_corner_points(image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([50, 80, 40])
        upper_green = np.array([80, 255, 100])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green) / 255
        print(green_mask.max(), green_mask.min(), green_mask.shape)
        #cv2.imshow('green_mask', green_mask)
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('gray', gray)
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray, 2, 3, 0.04)
        dst = cv2.dilate(dst,None)
        dst = dst * green_mask
        dst = dst > 0.01*dst.max()
        corner_points = np.argwhere(dst > 0.5)
        
        
        print('Number of corners found', len(corner_points))
        if len(corner_points) < 10000:
            point_index_to_cluster_id = TrackerOrientation.get_point_index_to_cluster_id(corner_points, 40)
            point_indices = TrackerOrientation.get_point_indices(point_index_to_cluster_id, 4)
            mean_points = TrackerOrientation.get_mean_points(point_indices, corner_points)
            print('Number of mean corners found', len(mean_points))
            filtered_mean_points = []
            for point_idx in range(len(mean_points)):
                point = mean_points[point_idx]
                if green_mask[int(point[1]), int(point[0])] > 0.5:
                    filtered_mean_points.append(point)
                    #print('green', hsv_image[int(point[1]), int(point[0]), :])
                    cv2.circle(image, (int(point[0]), int(point[1])), 5, color=(255, 0, 0), thickness=cv2.FILLED)
            return np.array(filtered_mean_points)
        return None

    @staticmethod
    def get_square_points(points):
        if points.shape[0] == 4:
            sorted_indices_x = np.argsort(points[:,0])
            sorted_indices_y = np.argsort(points[:,1])
            top_left_index  = set(sorted_indices_x[0:2]).intersection(set(sorted_indices_y[0:2]))
            top_right_index = set(sorted_indices_x[2:4]).intersection(set(sorted_indices_y[0:2]))
            bottom_left_index  = set(sorted_indices_x[0:2]).intersection(set(sorted_indices_y[2:4]))
            bottom_right_index  = set(sorted_indices_x[2:4]).intersection(set(sorted_indices_y[2:4]))
            if(len(top_left_index) == 1 and len(top_right_index) == 1 and len(bottom_left_index) == 1 and len(bottom_right_index) == 1):
                if len(top_left_index.union(top_right_index).union(bottom_left_index).union(bottom_right_index)) :
                    return SquarePoints(
                        points[list(top_left_index)[0]], 
                        points[list(top_right_index)[0]], 
                        points[list(bottom_left_index)[0]], 
                        points[list(bottom_right_index)[0]])
        return None

    def get_orientation_matrix(self, square_points):
        homography, status = cv2.findHomography(\
                square_points.correspondences(),
                square_points.points(), method=0)
        transformed_points = (homography @ square_points.correspondences().T)
        for point_idx in range(transformed_points.shape[1]):
            transformed_points[:, point_idx] = transformed_points[:, point_idx] / transformed_points[2, point_idx]
        reconstruction_error = transformed_points - square_points.points().T
        reconstruction_error = (reconstruction_error ** 2).mean()
        original_homography = self.inverse_camera_matrix @ homography
        r1 = original_homography[:,0] / np.linalg.norm(original_homography[:,0])
        r2 = original_homography[:,1] / np.linalg.norm(original_homography[:,0])
        r3 = np.cross(r1, r2)
        t = original_homography[:,2] / np.linalg.norm(original_homography[:,0])
        matrix =  np.stack([r1, r2, r3, t], axis=1)
        return matrix, reconstruction_error

    def get_pan_and_translation(matrix):
        r3 = matrix[:, 2]
        pan = np.arctan( r3[0] / r3[2] )
        tilt = np.arcsin(r3[1])
        t = matrix[:, 3]
        return pan, np.array([t[2], t[0], t[1]])

    def get_camera_orientation(self, image):
        mean_points = TrackerOrientation.get_corner_points(image)
        pan, t, error = None, None, None
        if mean_points is not None:
            square_points = TrackerOrientation.get_square_points(mean_points)
            if square_points is not None:
                matrix, error = self.get_orientation_matrix(square_points)
                if error < 1e-5:
                    self.last_square_points = square_points
                    pan, t = TrackerOrientation.get_pan_and_translation(matrix)
        if self.last_square_points is not None:
            distances = np.array([ TrackerOrientation.point_distance(self.last_square_points.top_left, 
                                self.last_square_points.top_right),
                          TrackerOrientation.point_distance(self.last_square_points.top_left, 
                                self.last_square_points.bottom_left),
                          TrackerOrientation.point_distance(self.last_square_points.top_left, 
                                self.last_square_points.bottom_right)])
            if distances.std() < 60 and distances.std() < 200:
                #print('distances', distances.mean(), distances.std())
                cv2.circle(image, (int(self.last_square_points.top_left[0]), int(self.last_square_points.top_left[1])), \
                    5, color=(0, 0, 255), thickness=cv2.FILLED)
                cv2.circle(image, (int(self.last_square_points.top_right[0]), int(self.last_square_points.top_right[1])), \
                    5, color=(0, 0, 255), thickness=cv2.FILLED)
                cv2.circle(image, (int(self.last_square_points.bottom_left[0]), int(self.last_square_points.bottom_left[1])), \
                    5, color=(0, 0, 255), thickness=cv2.FILLED)
                cv2.circle(image, (int(self.last_square_points.bottom_right[0]), int(self.last_square_points.bottom_right[1])), \
                    5, color=(0, 0, 255), thickness=cv2.FILLED)
                if pan is not None:
                    return pan, t, error, image
        return None

if __name__ == '__main__':
    CAMERA_MATRIX_FILENAME = "../camera/camera_matrix.numpy"
    DISTORITION_COEFFICIENTS_FILENAME = "../camera/distortion_coefficients.numpy"
    camera_matrix = None
    if os.path.exists(CAMERA_MATRIX_FILENAME) and os.path.exists(DISTORITION_COEFFICIENTS_FILENAME):
            with open(CAMERA_MATRIX_FILENAME, "rb") as camera_matrix_file:
                camera_matrix = np.load(camera_matrix_file)
            with open(DISTORITION_COEFFICIENTS_FILENAME, "rb") as dist_file:
                distortion_coefficients = np.load(dist_file)

    inverse_camera_matrix = np.linalg.inv(camera_matrix)
    tracker_orientation = TrackerOrientation(inverse_camera_matrix)
    video_capture = cv2.VideoCapture(4)
    #for image_file in ['0.jpg', '1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg']:
    while video_capture.isOpened():
        success, image = video_capture.read()
        #image = cv2.imread(f'/home/rusalka/Pictures/Webcam/{image_file}')
        something = tracker_orientation.get_camera_orientation(image)
        cv2.imshow('dst', image)
        if something is not None:
            pan, t, error, image = something
            print('error', error, 't', t, 'pan', pan)
        if cv2.waitKey(33) & 0xff == 27:
            cv2.destroyAllWindows()