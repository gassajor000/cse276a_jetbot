"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""

from filterpy.kalman import KalmanFilter
import numpy

class PositionDetector:
    # Number of landmarks position estimate is based off of
    CONFIDENCE_0LM = 0
    CONFIDENCE_1LM = 1
    CONFIDENCE_2LM = 2
    CONFIDENCE_3LM = 3

    MOVEMENT_NOISE = 0.1   # +/- 10 cm on driving
    ROTATION_NOISE = numpy.deg2rad(20)   # +/- 20 degrees on rotation
    MEASUREMENT_NOISE = 0.05     # Assume distance measurements are +/- 5 cm

    def __init__(self, init_pos=(0.0, 0.0, 0.0)):
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
        u vector     distance
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
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
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

        # todo setup camera & object detection
        self.detector = self.LandmarkDetector()
        self.locator = self.PositionLocater()

    def _make_B_vector(self):
        return numpy.array([
            [numpy.cos(self.filter.x[2]), 0],
            [numpy.sin(self.filter.x[2]), 0],
            [0, 1]
        ])

    def get_position(self, distance, rotation):
        """
        Get the estimated position of the robot
        :return: tuple (x, y, theta) position & orientation of the robot
        """
        # predict
        self.filter.predict(u=numpy.array([distance, rotation]), B=self._make_B_vector())

        # TODO: measure location using camera
        z = self.locator.get_position_from_landmarks(self.detector.detect_landmarks(), tuple(self.filter.x))
        self.filter.update(z=z)

        return self.filter.x

    class LandmarkDetector():
        # Landmark positions & classes
        LANDMARKS = {'landmark1': (0,0), 'landmark2': (0,0), 'landmark3': (0,0), 'landmark4': (0,0)}

        CAMERA_OFFSET = 2.0  # cm between the camera and the position model point

        def calibrate(self):
            """
            for each landmark, measure bounding box at 0.1, 0.5, 1.0, 1.5, and 2.0 meters
            print results to screen for computation
            """

        def detect_landmarks(self):
            """
            Return a list of all landmarks currently visible
            """
            # TODO use current position / orientation to eliminate false positives?

        def get_landmark(self, detection):
            """
            returns the landmark corresponding to the object detected.
            Returns None if object does not match any landmarks

            :param detection: object detection returned from jetbot Object Detector
            """
            return self.LANDMARKS.get(detection['label'], None)

        def get_distance_to_landmark(self, landmark, detection):
            """
            :param landmark: one of the landmarks in self.LANDMARKS
            :param detection: object detection returned from jetbot Object Detector
            :return: the estimated distance to landmark
            """
            # Compute height in pixels
            # Use similar triangles to estimate distance


    class PositionLocater():
        def get_position_from_landmarks(self, landmark_dists, estimated_position):
            """
            Estimate the position using relative distances to landmarks.
            :param landmark_dists: list of tuples containing a landmark and a distance measurement.
            :return: tuple, (x, y, confidence) of estimated position
            """
            pass
