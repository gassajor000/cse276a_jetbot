"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
import math
import random
import subprocess
import uuid

from filterpy.kalman import KalmanFilter
import numpy
import cv2
import glob

from jetbot import ObjectDetector, Camera, bgr8_to_jpeg


def dist(x1, y1, x2, y2):
    # distance between 2 points
    return numpy.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

class PositionDetector:
    MOVEMENT_NOISE = 0.1   # +/- 10 cm on driving
    ROTATION_NOISE = numpy.deg2rad(20)   # +/- 20 degrees on rotation
    MEASUREMENT_NOISE = 0.05     # Assume distance measurements are +/- 5 cm
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position

    def __init__(self, init_pos=(0.0, 0.0, 0.0), model_path='/home/jetbot/Notebooks/object_following/'):
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

        self.detector = self.LandmarkDetector()
        self.locator = self.PositionLocater(self.MEASUREMENT_NOISE, self.ESTIMATION_PROXIMITY)

        # Setup camera & object detection
        self.model = ObjectDetector(model_path + 'ssd_mobilenet_v2_coco.engine')
        self.camera = Camera.instance(width=300, height=300)

        # TODO camera calibration
    def calibrate_camera(self, file_path):
        """Run camera calibration on captured images"""
        print('Beginning Camera Calibration')
        # camera calibration
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = numpy.zeros((6 * 9, 3), numpy.float32)
        objp[:, :2] = numpy.mgrid[0:9, 0:6].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        images = glob.glob(file_path + '*.jpg')
        gray = None
        for fname in images:
            print('Processing Image ' + fname)
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (6, 9), None)
            print('found ret {}'.format(ret, corners))
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (6, 9), corners2, ret)
                cv2.imwrite(fname + '_processed.jpg', img)
        print("obj size {}, img size {}".format(len(objpoints), len(imgpoints)))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.mtx = mtx
        print('Camera Matrix ', mtx)

    def calibrate(self, file_path='images/'):
        """Calibrate capture images, calibrate camera, calibrate detector"""
        self.calibrate_camera(file_path)
        self.detector.calibrate(self.model, self.camera)

    def close(self):
        """Clean up resources and extra threads"""
        self.camera.stop()

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

        # undistort image
        img = cv2.imread('left12.jpg')
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, dist, (w, h), 1, (w, h))
        mapx, mapy = cv2.initUndistortRectifyMap(self.mtx, dist, None, newcameramtx, (w, h), 5)
        dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]
        cv2.imwrite('calibresult.png', dst)

        # measure location using camera
        detections = self.model(dst)
        x1, y1 = self.locator.get_position_from_landmarks(self.detector.detect_landmarks(detections), tuple(self.filter.x))
        theta = None    # TODO get orientation
        self.filter.update(z=(x1, y1, theta))

        return self.filter.x

    class LandmarkDetector():
        class Landmark():
            def __init__(self, position, height, name, category, label):
                self.label = label
                self.name = name
                self.category = category
                self.height = height    # cm
                self.position = position

        # Landmark positions & classes
        LANDMARKS = {0: Landmark((0,0), 0, 'landmark0', 'lmk', 0), 1: Landmark((0,0), 0, 'landmark1', 'lmk', 1),
                     2: Landmark((0,0), 0, 'landmark2', 'lmk', 2), 3: Landmark((0,0), 0, 'landmark3', 'lmk', 3)}

        CAMERA_OFFSET = 2.0  # cm between the camera and the position model point
        FOCAL_LENGTH = .0315     # 3.15 cm
        PIXEL_SIZE = 0.000112   # 1.12 um /pixel
        RAD_PER_PIXEL = math.radians(136) / 300   # degrees offset from straight on per pixel offset

        def calibrate(self, object_detector, camera):
            """
            For each landmark, measure bounding box at 0.1, 0.5, 1.0, 1.5, and 2.0 meters
            print results to screen for computation
            """
            def get_focal_length():
                return d * height_on_sensor_cm / lmk[0].height

            print('Beginning LandmarkDetector Calibration')
            lmk = self.LANDMARKS[0]
            focal_lengths = []
            for d in [0.1, 0.5, 1.0, 1.5, 2.0]:
                input("Place Landmark 0 {:.2f}m from robot then press any key to continue".format(d))

                detections = object_detector(camera.value)
                landmarks = self.detect_landmarks(detections)
                lmk = landmarks.get(0)
                f = []

                if lmk is None:
                    raise RuntimeError('Landmark 0 was not detected')

                height_on_sensor_cm = self.get_height_on_sensor(lmk[1])
                focal_lengths.append(get_focal_length())

            f = float(numpy.mean(focal_lengths))
            print('Average focal length = {:.3f}'.format(f))
            self.FOCAL_LENGTH = f

        def get_height_on_sensor(self, detection):
            """Return the height of the detection on the sensor in cm"""
            height_pixels = detection[1]['y1'] - detection[1]['y2']
            return self.PIXEL_SIZE * height_pixels

        def get_offset_from_center(self, detection):
            """Return the number of pixels detection is offset from center. Positive = right/ negative = left"""
            width_pixels = detection[1]['x1'] - detection[1]['x2']
            obj_center = detection[1]['x1'] - width_pixels/2
            return obj_center - 150

        def detect_landmarks(self, detections):
            """
            Return a list of all landmarks currently visible, paired with the detection data, keyed by the landmark label
            """
            landmarks = {}
            for det in detections:
                lmk = self._get_landmark(det)
                if lmk is not None:
                    landmarks[lmk.label] = ((lmk, det))

            return landmarks

        def _get_landmark(self, detection):
            """
            returns the landmark corresponding to the object detected.
            Returns None if object does not match any landmarks

            :param detection: object detection returned from jetbot Object Detector
            """
            return self.LANDMARKS.get(detection['label'], None)

        def get_distance_to_landmark(self, landmark: Landmark, detection):
            """
            :param landmark: one of the landmarks in self.LANDMARKS
            :param detection: object detection returned from jetbot Object Detector
            :return: the estimated distance to landmark (in meters)
            """

            height_on_sensor_cm = self.get_height_on_sensor(detection)

            # Use similar triangles to estimate distance
            d_cm = landmark.height * self.FOCAL_LENGTH / height_on_sensor_cm

            return d_cm / 100   # convert to meters

        def get_angle_offset_to_landmark(self, detection):
            """
            Get the angle offset from the direction the robot is looking to the object
            :param detection: object detection returned from jetbot Object Detector
            :return: offset (radians). Positive = right, Negative = left
            """
            return self.get_offset_from_center(detection) * self.RAD_PER_PIXEL

    class PositionLocater():
        # Number of landmarks position estimate is based off of
        CONFIDENCE_0LM = 0
        CONFIDENCE_1LM = 1
        CONFIDENCE_2LM = 2
        CONFIDENCE_3LM = 3

        PARTICLE_BATCH_SIZE = 150


        def __init__(self, measurement_noise, estimation_proximity_threshold):
            self.estimation_proximity_threshold = estimation_proximity_threshold
            self.measurement_noise = measurement_noise

        class Particle():
            def __init__(self, x, y):
                self.x = x
                self.y = y
                self.num_rings = 0

            @staticmethod
            def from_ring(origin, radius, max_err):
                rand_r = radius + random.uniform(-max_err, max_err)
                theta = random.uniform(0, 2*numpy.pi)
                x = origin[0] + rand_r* math.cos(theta)
                y = origin[1] + rand_r* math.sin(theta)

                return PositionDetector.PositionLocater.Particle(x, y)


            def is_inside_rings(self, origin, r_outer, r_inner):
                # True if particle lies inside the two rings, False otherwise
                d = dist(self.x, self.y, origin[0], origin[1])
                return d <= r_outer and d >= r_inner

            def __str__(self):
                return "({:.4f}, {:.4f}, {})".format(self.x, self.y, self.num_rings)

            def __repr__(self):
                return "Particle " + str(self)

        def get_position_from_landmarks(self, landmark_dists, estimated_position):
            """
            Estimate the position using relative distances to landmarks.
            :param landmark_dists: list of tuples containing a landmark and a distance measurement.
            :param estimated_position: (x, y) of the robot's estimated position
            :return: tuple, ((x, y), confidence) of estimated position
            """
            def plot(particles):     # for debugging
                from matplotlib import pyplot
                color = {0: 'red', 1:'blue', 2:'green', 3: 'yellow', 4: 'orange'}
                x = list(map(lambda p: p.x, particles))
                y = list(map(lambda p: p.y, particles))
                colors = list(map(lambda p: color[p.num_rings], particles))
                circles = list(map(lambda lmk: pyplot.Circle((lmk[0][0], lmk[0][1]), lmk[1], fill=False), landmark_dists))
                ax = pyplot.gca()
                ax.set_xlim((0, 3))
                ax.set_ylim((0, 3))
                ax.scatter([estimated_position[0]], [estimated_position[1]], marker='*')
                ax.scatter(x, y, c=colors)
                for circle in circles:
                    ax.add_artist(circle)
                pyplot.show()


            if len(landmark_dists) == 0:
                return estimated_position, self.CONFIDENCE_0LM  # Your guess is as good as mine
            elif len(landmark_dists) == 1:
                # return closest particle on ring to estimated position
                origin = landmark_dists[0][0][0], landmark_dists[0][0][1]
                r = landmark_dists[0][1]
                v = estimated_position[0] - origin[0], estimated_position[1] - origin[1]
                d = dist(estimated_position[0], estimated_position[1], origin[0], origin[1])
                a = origin[0] + v[0]/d * r,  origin[1] + v[1]/d * r
                return a, self.CONFIDENCE_1LM

            # generate a list of particles
            particles = []
            for lmk in landmark_dists:
                particles += [self.Particle.from_ring(lmk[0], lmk[1], self.measurement_noise) for _ in range(self.PARTICLE_BATCH_SIZE)]

            # record the number of rings each particle is in range in
            most_rings = 0      # keep track of the most number of rings particles fall in
            for lmk in landmark_dists:
                #           x, y, radius
                ring_outer = lmk[1] + self.measurement_noise
                ring_inner = lmk[1] - self.measurement_noise

                for particle in particles:
                    if particle.is_inside_rings(lmk[0], ring_outer, ring_inner):
                        particle.num_rings += 1

                    if particle.num_rings > most_rings:    # update most rings
                        most_rings = particle.num_rings

            if most_rings == 1:
                # no overlap between rings, error out
                return None, None

            best_particles = list(filter(lambda p: p.num_rings == most_rings, particles))
            # plot(best_particles)  # Uncomment for debug plot

            if most_rings < len(landmark_dists) or most_rings == 2:    # did not have an ideal reading/overlap zone (likely a bad distance measurement)
                # use estimated position to narrow particles
                best_particles = filter(lambda p: p.is_inside_rings(estimated_position, self.estimation_proximity_threshold, 0), best_particles)

            coords = list(map(lambda p: (p.x, p.y), best_particles))  # convert to tuples for numpy

            if len(coords) == 0:    # No particles were close to the estimated position!
                return None, None

            center = numpy.mean(list(coords), axis=0)
            return tuple(center), self.CONFIDENCE_2LM if most_rings == 2 else self.CONFIDENCE_3LM

        def get_orientation_from_landmarks(self, landmark_thetas, position):
            """
            Find the orientation from the detected landmarks and detected position
            :param landmark_dists: list of tuples containing a landmark and the robot's orientation offset from the landmark
            :param position: (x,y) position returned from get_position_from_landmarks()
            :return: global orientation (theta) in radians
            """
            def get_theta_to_obj(landmark):
                """use position to determine the global rotation to see the object head on"""
                dy = position[1] - landmark.position[1]
                dx = position[0] - landmark.position[0]
                theta = math.atan(dy/dx)

                if dy >= 0:
                    if dx >= 0:
                        return math.pi/2 - theta    # Q1
                    else:
                        return 3*math.pi/2 + theta    # Q2

                else:
                    if dx >= 0:
                        return math.pi/2 + theta    # Q4
                    else:
                        return 3*math.pi/2 - theta    # Q3


            global_thetas = []
            for landmark, theta in landmark_thetas:
                # compute global thenta to landmark (observed theta looking North)
                landmark_global_theta = get_theta_to_obj(landmark)

                global_thetas.append(landmark_global_theta - theta)     # global - offset

            # remove outliers
            g_avg = float(numpy.mean(global_thetas))
            g_std = float(numpy.std(global_thetas))
            for val in global_thetas:
                z = (val - g_avg) / g_std
                if abs(z) > 2:      # try to remove bad readings (false positive detections)
                    global_thetas.remove(val)

            # average
            return float(numpy.mean(global_thetas))