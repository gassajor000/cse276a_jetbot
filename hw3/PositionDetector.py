"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
from filterpy.kalman import KalmanFilter
import numpy

from hw3.Camera import Camera
from hw3.LandmarkDetector import LandmarkDetector
from hw3.PositionTriangulator import PositionTriangulator


class PositionDetector:
    MOVEMENT_NOISE = 0.1   # +/- 10 cm on driving
    ROTATION_NOISE = numpy.deg2rad(20)   # +/- 20 degrees on rotation
    MEASUREMENT_NOISE = 0.05     # Assume distance measurements are +/- 5 cm
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position

    def __init__(self, init_pos=(0.0, 0.0, 0.0), model=None, camera_instance=None):
        """
        Initialize camera, object detection and Kalman Fiilter


        :param init_pos: initial position (x , y, theta)

        Kalman Filter
        filter.x = (x, y, theta)
        filler.z = (x, y, theta)
        filter.F =  1 0 0
                    0 1 0
                    0 0 1
        filter.H =  1 0 0
                    0 1 0
                    0 0 1
        u vector    distance
                    rotation
        filter.B =  cos(theta) 0
                    sin(theta) 0
                    0          1
        filter.P =  5 0 0
                    0 5 0
                    0 0 5
        filter.Q =  M 0 0
                    0 M 0
                    0 0 R
        filter.R =  E 0 0
                    0 E 0
                    0 0 E
        """
        # setup kalman filter
        ident = numpy.array([
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 1.],
        ])
        self.filter = KalmanFilter(dim_x=3, dim_z=3)
        self.filter.x = numpy.array([[init_pos[0], init_pos[1], init_pos[2]]])
        self.filter.F = ident
        self.filter.H = ident
        self.filter.P = ident * 5
        self.filter.Q = numpy.array([
            [self.MOVEMENT_NOISE, 0, 0,],
            [0, self.MOVEMENT_NOISE, 0,],
            [0, 0, self.ROTATION_NOISE,],
        ])
        self.filter.R = ident * self.MEASUREMENT_NOISE

        self.detector = LandmarkDetector(model=model)
        self.locator = PositionTriangulator(self.MEASUREMENT_NOISE, self.ESTIMATION_PROXIMITY)
        self.camera = Camera(camera_instance)

    def calibrate(self, file_path='images/'):
        """Calibrate capture images, calibrate camera, calibrate detector"""
        self.camera.calibrate(file_path)
        self.detector.calibrate(self.camera.get_image)

    def close(self):
        """Clean up resources and extra threads"""
        self.camera.close()

    def _make_B_vector(self):
        return numpy.array([
            [numpy.cos(self.filter.x[2]), 0.],
            [numpy.sin(self.filter.x[2]), 0.],
            [0., 1.]
        ])

    def get_position(self, distance, rotation):
        """
        Get the estimated position of the robot
        :return: tuple (x, y, theta) position & orientation of the robot
        """
        # predict
        self.filter.predict(u=numpy.array([distance, rotation]), B=self._make_B_vector())
        print("Predicted Position ({:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        image = self.camera.get_image()
        landmark_detections = self.detector.detect_landmarks(image)

        if not landmark_detections:
            return self.filter.x    # Couldn't find any landmarks. Just use the prediction.

        landmark_distances = list(map(lambda lmk: self.detector.get_distance_to_landmark(**lmk), landmark_detections))
        landmark_angles = list(map(lambda lmk: self.detector.get_angle_offset_to_landmark(**lmk), landmark_detections))

        x1, y1 = self.locator.get_position_from_landmarks(landmark_distances, tuple(self.filter.x))
        theta = self.locator.get_orientation_from_landmarks(landmark_angles, tuple(self.filter.x))
        print("Computed Position ({:.3f}, {:.3f}, {:.3f})".format(x1, y1, theta))

        self.filter.update(z=(x1, y1, theta))
        print("Updated Position ({:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        return self.filter.x
