"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
from filterpy.kalman import KalmanFilter
import numpy

from Camera import Camera

from LandmarkDetector import LandmarkDetector


class PositionDetector:
    V_NOISE = 0.03   # +/- 5cm/s on driving
    W_NOISE = numpy.deg2rad(20)   # +/- 20/s degrees on rotation
    D_NOISE = 0.05     # +/- 5 cm on d measurements
    P_NOISE = numpy.deg2rad(5)     # +/- 5 degrees on phi measurements
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position
    NUM_KINEMATIC_VARS = 5

    def __init__(self, landmark_detector: LandmarkDetector, init_pos=(0.0, 0.0, 0.0), model=None, camera_instance=None):
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
        self.detector = landmark_detector
        self.camera = Camera(camera_instance)

        self.num_landmarks = self.detector.get_num_landmarks()
        # keep an ordered list of the landmarks to keep track of their order in the x matrix
        self.landmarks_ids = self.detector.get_landmark_labels()
        self.num_vars = self.NUM_KINEMATIC_VARS + 2* self.num_landmarks   # (x, y, theta, v, omega, (x,y) for each landmark)

        # setup kalman filter
        self.filter = KalmanFilter(dim_x=self.num_vars, dim_z=self.num_landmarks*2)
        init_x = numpy.zeros(self.num_vars, dtype=float)
        init_x[0], init_x[1], init_x[2] = init_pos[0], init_pos[1], init_pos[2]
        init_x[3], init_x[4] = 0.0, 0.0     # initial velocity and omega

        landmark_positions = self.detector.get_landmark_positions()
        for i in range(self.num_landmarks):
            pos = landmark_positions[self.landmarks_ids[i]]
            k = 2 * i
            # landmark position
            init_x[k + self.NUM_KINEMATIC_VARS] = pos[0]
            init_x[k + self.NUM_KINEMATIC_VARS + 1] = pos[1]

        print('init x {} init pos {}'.format(init_x, init_pos))
        self.filter.x = init_x

        p = numpy.zeros((self.num_vars, self.num_vars))
        p[0][0] = 0.01
        p[1][1] = 0.01
        p[2][2] = 0.1
        self.filter.P = p * .25

        self.B = numpy.zeros((self.num_vars, 2), dtype=float)
        # v' = v  + a * dt
        self.B[0][0] = 1
        # w' = w + alpha * dt
        self.B[1][1] = 1

        self.Q = numpy.zeros((self.num_vars, self.num_vars), dtype=float)
        # x' = x  -v sin(theta) dt
        self.Q[0][0] = 0.5 * (self.V_NOISE) **2
        # y' = y + v cos(theta) dt
        self.Q[1][1] = 0.5 * (self.V_NOISE) **2
        # theta' = theta + omega dt
        self.Q[2][2] = (self.W_NOISE) ** 2
        self.Q[3][3] = (self.V_NOISE) ** 2
        self.Q[2][2] = (self.W_NOISE) ** 2

    def calibrate(self, file_path='images/'):
        """Calibrate capture images, calibrate camera, calibrate detector"""
        self.camera.calibrate(file_path)
        self.detector.calibrate(self.camera.get_image)

    def close(self):
        """Clean up resources and extra threads"""
        self.camera.close()

    def _get_B_matrix(self, dt):
        return self.B * dt

    def _get_F_matrix(self, dt):
        F = numpy.eye(self.num_vars)

        # x' = x - v * sin(theta) dt
        F[0][3] = -1 * numpy.sin(self.filter.x[2]) * dt
        # y' = y + v* cos(theta) dt
        F[1][3] = numpy.cos(self.filter.x[2]) * dt

    def _get_Q_matrix(self, dt):
        return self.Q * dt

    def _get_R_matrix(self):
        R = numpy.zeros((2*self.num_landmarks, 2*self.num_landmarks), dtype=float)
        for i in range(self.num_landmarks):
            k = 2*i
            # landmark position
            R[k][k] = (self.D_NOISE * numpy.cos(self.filter.x[2]) * self.P_NOISE) **2
            R[k+1][k+1] = (self.D_NOISE * numpy.sin(self.filter.x[2]) * self.P_NOISE) **2

        return R

    def _get_H_matrix(self, measurements):
        num_lm_coords = 2 * self.num_landmarks
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

    def force_v_omega_to_zero(self):
        if self.filter.x[3] < 0.02:
            self.filter.x[3] = 0.0
        if abs(self.filter.x[4]) < 0.1:
            self.filter.x[4] = 0.0

    def get_position(self, acceleration, alpha, dt, read_sensors=True):
        """
        Get the estimated position of the robot
        :param velocity: forward velocity of the robot (rel to robot, m/s)
        :param omega: angular velocity of the robot (rad/s)
        :param dt: time delta (time since last update)
        :return: tuple (x, y, theta, v, omega) position, orientation, speed, and angular speed of the robot
        """
        # predict
        self.filter.predict(u=numpy.array([acceleration, alpha]), B=self._get_B_matrix(dt), Q=self._get_Q_matrix(dt), F=self._get_F_matrix(dt))
        self.wrap_theta()
        self.force_v_omega_to_zero()
        if self.logging:
            print("Predicted Position ({:.3f}, {:.3f}, {:.3f}) [{:.3f}, {:.3f}]".format(self.filter.x[0], self.filter.x[1],
                                                                                        self.filter.x[2], self.filter.x[3], self.filter.x[4]))

        if read_sensors:
            image = self.camera.get_image()
            landmark_detections = self.detector.detect_landmarks(image)

            if not landmark_detections:
                return self.filter.x    # Couldn't find any landmarks. Just use the prediction.

            measurements = []
            for i in range(self.num_landmarks):
                lmk_id = self.landmarks_ids[i]
                if lmk_id in landmark_detections:
                    print('Detected {}'.format(lmk_id))
                    lmk, det = landmark_detections[lmk_id]
                    d = self.detector.get_distance_to_landmark(lmk, det)
                    phi = self.detector.get_angle_offset_to_landmark(det)
                    measurements.append((d, phi))

                else:
                    measurements.append(None)   # Landmark was not detected

            if self.logging:
                print("Relative Positions {}".format(measurements))

            self.filter.update(z=self.get_z_vector(measurements), H=self._get_H_matrix(measurements), R=self._get_R_matrix())
            if self.logging:
                print("Updated Position ({:.3f}, {:.3f}, {:.3f})".format(*self.filter.x))

        return self.filter.x

    def get_x_y_from_d_phi(self, d, phi):
        theta = self.filter.x[2]
        return -d *numpy.sin(theta + phi), d *numpy.cos(theta + phi)

    def get_z_vector(self, d_phi_measurements):
        """
        Produce a z vector from a list of (d, phi) measurements for the landmarks.
        :param d_phi_measurements: list of d, phi measurements for each landmark
        :return: 2x 2* num_landmarks vector of x,y positions, relative to the robot.
        """
        z_vec = numpy.zeros(2*self.num_landmarks, dtype=numpy.float)
        for i in range(self.num_landmarks):
            k = i*2
            if d_phi_measurements[i] is not None:
                d, phi = d_phi_measurements[i]
                x, y = self.get_x_y_from_d_phi(d, phi)
                z_vec[k] = x
                z_vec[k+1] = y
            else: # Nulled out by H anyways
                z_vec[k] = 0.0
                z_vec[k+1] = 0.0

        return z_vec

    def dump_x(self):
        print(self.landmarks_ids)
        print(self.filter.x)
