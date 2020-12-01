"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy

from Camera import Camera
from QRDetector import QRDetector


class PositionDetector:
    V_NOISE = 0.03   # +/- 5cm/s on driving
    W_NOISE = numpy.deg2rad(20)   # +/- 20/s degrees on rotation
    D_NOISE = 0.05     # +/- 5 cm on d measurements
    P_NOISE = numpy.deg2rad(5)     # +/- 5 degrees on phi measurements
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position
    NUM_KINEMATIC_VARS = 3

    def __init__(self, init_pos=(0.0, 0.0, 0.0), model=None, camera_instance=None):
        """
        Initialize camera, object detection and Kalman Fiilter

        :param init_pos: initial position (x , y, theta)

        Kalman Filter
        filter.x = (x, y, theta, xl1, yl1, ... xln, yln)
        filler.z = (xr1, yr1, ... xrn, yrn)
        filter.F =  1 0 0 0 0 ... 0 0
                    0 1 0 0 0     ...
                    0 0 1 0 0
                    0 0 0 0 1     0 0
                    ...           ...
                    0 0 0 0 0 ... 0 1

        filter.H =  -1 0 0 1 0 ... 0 0
                    0 -1 0 0 1 ... 0 0
                    ...            ...
                    -1 0 0 0 . ... 1 0
                    0 -1 0 0 . ... 0 1

        u vector    (velocity, omega)

        filter.B =  -sin(theta)dt   0
                    cos(theta) dt   0
                    0               0
                    0               dt
                    ...
                    0               0

        filter.P =  0.25 0 0 ... 0
                    0 0.25 0 ... 0
                    0 0 0.25 ... 0
                    0 0    0 ... 0.25

        filter.Q =  (sv sin(theta) dt)^2  0 0 0 0 0...0 0
                    0 (sv cos(theta) dt)^2 0 0 0 0 0
                    0 0 (sw dt)^2 0 0 0 0
                    0 0 0 P 0 0 0
                    0 0 0 0 P 0 0
                    0 0 0 0 0 0 0
                    ...
                    0 0 0 0 0 0 0...0 0

        filter.R =  (sd cos(theta) sp)^2 0 ... 0 0
                    0 (sd sin(theta) sp)^2 ... 0 0
                    ...                        ...
                    0 0                        (sd cos(theta) sp)^2 0
                    0 0                        0 (sd sin(theta) sp)^2
        """
        self.logging = False
        self.detector = QRDetector()
        self.camera = Camera(camera_instance)

        self.num_landmarks = self.detector.get_num_landmarks()
        # keep an ordered list of the landmarks to keep track of their order in the x matrix
        self.landmarks_ids = self.detector.get_landmark_labels()
        self.num_vars = self.NUM_KINEMATIC_VARS + 2* self.num_landmarks   # (x, y, theta, v, omega, (x,y) for each landmark)

        # setup kalman filter
        self.filter = KalmanFilter(dim_x=self.num_vars, dim_z=self.num_landmarks*2)
        init_x = numpy.zeros(self.num_vars, dtype=float)
        init_x[0], init_x[1], init_x[2] = init_pos[0], init_pos[1], init_pos[2]
        self.filter.x = numpy.array(init_x)

        ident = numpy.eye(self.num_vars)
        self.filter.P = ident * .25

    def calibrate(self, file_path='images/'):
        """Calibrate capture images, calibrate camera, calibrate detector"""
        self.camera.calibrate(file_path)
        self.detector.calibrate(self.camera.get_image)

    def close(self):
        """Clean up resources and extra threads"""
        self.camera.close()

    def _get_B_matrix(self, dt):
        B = numpy.zeros((self.num_vars, 2), dtype=float)
        # x' = x  -v sin(theta) dt
        B[0][0] =  -numpy.sin(self.filter.x[2]) * dt
        # y' = y + v cos(theta) dt
        B[1][0] = numpy.cos(self.filter.x[2]) * dt
        # theta' = theta + omega dt
        B[2][1] = dt

        return B

    def _get_Q_matrix(self, dt):
        Q = numpy.zeros((self.num_vars, self.num_vars), dtype=float)
        # x' = x  -v sin(theta) dt
        Q[0][0] =  (-numpy.sin(self.filter.x[2]) * dt * self.V_NOISE) **2
        # y' = y + v cos(theta) dt
        Q[1][0] = (numpy.cos(self.filter.x[2]) * dt * self.V_NOISE) **2
        # theta' = theta + omega dt
        Q[2][1] = (dt * self.W_NOISE) ** 2

        return Q

    def _get_R_matrix(self):
        R = numpy.zeros((self.num_landmarks, self.num_landmarks), dtype=float)
        for i in range(self.num_landmarks):
            k = 2*i
            # landmark position
            R[k][k] = (self.D_NOISE * numpy.cos(self.filter.x[2]) * self.P_NOISE) **2
            R[k+1][k+1] = (self.D_NOISE * numpy.sin(self.filter.x[2]) * self.P_NOISE) **2

        return R

    def _get_H_matrix(self, measurements):
        num_lm_coords =  2* self.num_landmarks
        H = numpy.zeros((num_lm_coords, num_lm_coords + self.NUM_KINEMATIC_VARS), dtype=float)
        for i in range(self.num_landmarks):
            if measurements[i] is not None:     # do not include landmarks that were not detected
                k = 2*i
                # landmark position
                H[k][k+self.NUM_KINEMATIC_VARS] = 1.0
                H[k+1][k+self.NUM_KINEMATIC_VARS+1] = 1.0
                # subtract robot position
                H[k][0] = -1.0
                H[k+1][1] = -1.0

        return H

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
        self.filter.predict(u=numpy.array([velocity, omega]), B=self._get_B_matrix(dt), Q=self._get_Q_matrix(dt))
        self.wrap_theta()
        if self.logging:
            print("Predicted Position ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        image = self.camera.get_image()
        landmark_detections = self.detector.detect_landmarks(image)

        if not landmark_detections:
            if self.logging:
                print("No landmarks detected")
            return self.filter.x    # Couldn't find any landmarks. Just use the prediction.

        measurements = []
        for i in range(self.landmarks_ids):
            lmk_id = self.landmarks_ids[i]
            if lmk_id in landmark_detections:
                lmk, det = landmark_detections[lmk_id]
                d = (lmk.position, self.detector.get_distance_to_landmark(lmk, det))
                phi = lmk, self.detector.get_angle_offset_to_landmark(det)
                measurements.append((d, phi))
            else:
                measurements.append(None)   # Landmark was not detected

        if self.logging:
            print("Relative Positions {}".format(measurements))

        self.filter.update(z=self.get_z_vector(measurements), H=self._get_H_matrix(measurements))
        if self.logging:
            print("Updated Position ({:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        return self.filter.x

    def get_z_vector(self, d_phi_measurements):
        """
        Produce a z vector from a list of (d, phi) measurements for the landmarks.
        :param d_phi_measurements: list of d, phi measurements for each landmark
        :return: 2x 2* num_landmarks vector of x,y positions, relative to the robot.
        """
        theta = self.filter.x[2]

        def get_x_y(d, phi):
            return -d *numpy.sin(theta + phi), d *numpy.cos(theta + phi)

        z_vec = numpy.zeros(2*self.num_landmarks, dtype=numpy.float)
        for i in range(self.num_landmarks):
            k = i*2
            if d_phi_measurements[i] is not None:
                d, phi = d_phi_measurements[i]
                x, y = get_x_y(d, phi)
                z_vec[k] = x
                z_vec[k+1] = y
            else: # Nulled out by H anyways
                z_vec[k] = 0.0
                z_vec[k+1] = 0.0

        return z_vec

    def dump_x(self):
        print(self.filter.x)
