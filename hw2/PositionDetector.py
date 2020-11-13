"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""
import math
import random

from filterpy.kalman import KalmanFilter
import numpy


def dist(x1, y1, x2, y2):
    # distance between 2 points
    return numpy.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

class PositionDetector:
    MOVEMENT_NOISE = 0.1   # +/- 10 cm on driving
    ROTATION_NOISE = numpy.deg2rad(20)   # +/- 20 degrees on rotation
    MEASUREMENT_NOISE = 0.05     # Assume distance measurements are +/- 5 cm
    ESTIMATION_PROXIMITY = 0.25  # Bias position readings to within 25cm of predicted position

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

        # todo setup camera & object detection
        self.detector = self.LandmarkDetector()
        self.locator = self.PositionLocater(self.MEASUREMENT_NOISE, self.ESTIMATION_PROXIMITY)

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
