"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
import math
import random
import time

from filterpy.kalman import KalmanFilter
import numpy

from hw3.Camera import Camera
from jetbot import ObjectDetector


def dist(x1, y1, x2, y2):
    # distance between 2 points
    return numpy.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

class PositionDetector:
    MOVEMENT_NOISE = 0.1   # +/- 10 cm on driving
    ROTATION_NOISE = numpy.deg2rad(20)   # +/- 20 degrees on rotation
    MEASUREMENT_NOISE = 0.05     # Assume distance measurements are +/- 5 cm
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position

    def __init__(self, init_pos=(0.0, 0.0, 0.0), model_path='/home/jetbot/Notebooks/object_following/', model=None, camera_instance=None):
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
        if model:
            self.model = model
        else:
            print("Initializing Model...")
            self.model = ObjectDetector(model_path + 'ssd_mobilenet_v2_coco.engine')

        self.camera = Camera(camera_instance)


    def calibrate(self, file_path='images/'):
        """Calibrate capture images, calibrate camera, calibrate detector"""
        self.camera.calibrate(file_path)
        self.detector.calibrate(self.model, self.camera.get_instance())

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


        start_time = time.time()
        landmark_detections = None
        while time.time() < start_time + 5: # 5second timeout
            # undistort image
            image = self.camera.get_image()
            # measure location using camera
            detections = self.model(image)
            landmark_detections = self.detector.detect_landmarks(detections)

            if len(landmark_detections) > 1:    # try to get more than 1 landmark
                break
            time.sleep(.02) # Wait for a bit before sampling again

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

    class LandmarkDetector():
        class Landmark():
            def __init__(self, position, height, name, category, label):
                self.label = label
                self.name = name
                self.category = category
                self.height = height    # cm
                self.position = position

        # Landmark positions & classes
        LANDMARKS = {44: Landmark((0,0), 22.5, 'landmark 0 [olive oil]', 'bottle', 44),
                     53: Landmark((0,0), 7.5, 'landmark 1 [apple]', 'apple', 53),
                     32: Landmark((0,0), 150, 'landmark 2 [tie]', 'tie', 32),
                     51: Landmark((0,0), 11.5, 'landmark 3 [bowl]', 'bowl', 51),
                     13: Landmark((0,0), 23.0, 'landmark 4 [stop sign]', 'stop sign', 32),
                     85: Landmark((0,0), 13.6, 'landmark 5 [clock]', 'clock', 85)}

        CAMERA_OFFSET = 2.0  # cm between the camera and the position model point
        FOCAL_LENGTH = .159     # 1.59 mm
        PIXEL_SIZE = 0.0009199   # (cm) 9.199 um /pixel
        RAD_PER_PIXEL = math.radians(136) / 300   # degrees offset from straight on per pixel offset

        def calibrate(self, object_detector, camera):
            """
            For each landmark, measure bounding box at 0.1, 0.5, 1.0, 1.5, and 2.0 meters
            print results to screen for computation
            """
            def get_focal_length():
                return (d * 100) * height_on_sensor_cm / lmk.height

            print('Beginning LandmarkDetector Calibration')
            lmk = self.LANDMARKS[44]
            focal_lengths = []
            for d in [0.25, 0.5, 1.0, 1.25]:
                input("Place {} {:.2f}m from robot then press any key to continue".format(lmk.name, d))

                import time
                timeout = 5
                time_start = time.time()
                lmks = None
                while time.time() - time_start < timeout:
                    detections = object_detector(camera.value)
                    landmarks = self.detect_landmarks(detections[0])
                    lmk1, det = landmarks.get(lmk.label, (None, None))
                    if lmk1:
                        break
                    else:
                        time.sleep(.02)
                else:
                    raise RuntimeError('{} was not detected'.format(lmk.name))

                print(lmk1)
                height_on_sensor_cm = self.get_height_on_sensor(det)
                f = get_focal_length()
                print("height on sensor {}, f {}".format(height_on_sensor_cm, f))
                focal_lengths.append(f)


            f = float(numpy.mean(focal_lengths))
            print('Average focal length = {:.3f}'.format(f))
            self.FOCAL_LENGTH = f

        def get_height_on_sensor(self, detection):
            """Return the height of the detection on the sensor in cm"""
            x1, y1, x2, y2 = detection['bbox']
            height_pixels = abs(y1 - y2) * 300    # Scale to height of image
#             print("{} pixels high. bbox {}".format(height_pixels, detection['bbox']))
            return self.PIXEL_SIZE * height_pixels

        def get_offset_from_center(self, detection):
            """Return the number of pixels detection is offset from center. Positive = right/ negative = left"""
            x1, y1, x2, y2 = detection['bbox']
            obj_center = ((x1 + x2) / 2.0) * 300
#             print(obj_center)
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