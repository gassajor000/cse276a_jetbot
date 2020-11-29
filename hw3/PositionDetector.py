"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
from filterpy.kalman import KalmanFilter
import numpy

from Camera import Camera
from LandmarkDetector import LandmarkDetector
from PositionTriangulator import PositionTriangulator


class PositionDetector:
    MOVEMENT_NOISE = 0.1   # +/- 10 cm on driving
    ROTATION_NOISE = numpy.deg2rad(20)   # +/- 20 degrees on rotation
    MEASUREMENT_NOISE = 0.05     # Assume distance measurements are +/- 5 cm
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position

    def __init__(self, init_pos=(0.0, 0.0, 0.0), initial_v=0.0, initial_omega=0.0, model=None, camera_instance=None):
        """
        Initialize camera, object detection and Kalman Fiilter

        :param init_pos: initial position (x , y, theta)

        Kalman Filter
        filter.x = (x, y, theta, v, omega)
        filler.z = (x, y, theta)
        filter.F =  1 0 0 -sin(theta)dt 0
                    0 1 0 cos(theta)dt  0
                    0 0 1 0             dt
                    0 0 0 0             0
                    0 0 0 0             0
        filter.H =  1 0 0 0 0
                    0 1 0 0 0
                    0 0 1 0 0

        u vector    velocity
                    omega
        filter.B =  0 0
                    0 0
                    0 0
                    1 0
                    0 1
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
            [1., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0.],
            [0., 0., 1., 0., 0. ],
            [0., 0., 0., 1., 0. ],
            [0., 0., 0., 0., 1. ],
        ])
        self.filter = KalmanFilter(dim_x=5, dim_z=3)
        self.filter.x = numpy.array([init_pos[0], init_pos[1], init_pos[2], initial_v, initial_omega])
        self.filter.F = ident
        self.filter.H = numpy.array([
            [1., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0.],
            [0., 0., 1., 0., 0.],
        ])
        self.filter.P = ident * 5
        self.filter.Q = ident * self.MOVEMENT_NOISE
        self.filter.R = numpy.eye(3) * self.MEASUREMENT_NOISE
        self.filter.B =  numpy.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 1],
        ])
        self.logging = False

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

    def _make_F_matrix(self, dt):
        return numpy.array([
            [1, 0, 0, -numpy.sin(self.filter.x[2]) * dt, 0],
            [0, 1, 0, numpy.cos(self.filter.x[2]) * dt, 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ])

    def wrap_theta(self):
        theta = self.filter.x[2]
        pi_2 = 2 * numpy.pi
        if theta >= pi_2:
            self.filter.x[2] -= pi_2
        elif theta < 0:
            self.filter.x[2] += pi_2

    def get_position(self, velocity, omega, dt):
        """
        Get the estimated position of the robot
        :param velocity: forward velocity of the robot (rel to robot, m/s)
        :param omega: angular velocity of the robot (rad/s)
        :param dt: time delta (time since last update)
        :return: tuple (x, y, theta, v, omega) position, orientation, speed, and angular speed of the robot
        """
        # predict
        self.filter.predict(u=numpy.array([velocity, omega]), F=self._make_F_matrix(dt))
        self.wrap_theta()
        if self.logging:
            print("Predicted Position ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        image = self.camera.get_image()
        landmark_detections = self.detector.detect_landmarks(image)

        if not landmark_detections:
            if self.logging:
                print("No landmarks detected")
            return self.filter.x    # Couldn't find any landmarks. Just use the prediction.

        landmark_distances = []
        landmark_angles = []
        for lmk, det in landmark_detections.values():
            landmark_distances.append((lmk.position, self.detector.get_distance_to_landmark(lmk, det)))
            landmark_angles.append((lmk, self.detector.get_angle_offset_to_landmark(det)))

        pos, confidence = self.locator.get_position_from_landmarks(landmark_distances, tuple(self.filter.x))
        theta = self.locator.get_orientation_from_landmarks(landmark_angles, tuple(self.filter.x))
        if self.logging:
            print("Computed Position ({:.3f}, {:.3f}, {:.3f})".format(pos[0], pos[1], theta))

        self.filter.update(z=(pos[0], pos[1], theta))
        if self.logging:
            print("Updated Position ({:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        return self.filter.x
