"""
    created by Jordan Gassaway, 11/25/2020
    PositionTriangulator: locates position by triangulation from landmarks
"""
import math
import numpy
import random


class PositionTriangulator():
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
            theta = random.uniform(0, 2 * math.pi)
            x = origin[0] + rand_r * math.cos(theta)
            y = origin[1] + rand_r * math.sin(theta)

            return PositionTriangulator.Particle(x, y)

        def is_inside_rings(self, origin, r_outer, r_inner):
            # True if particle lies inside the two rings, False otherwise
            d = PositionTriangulator.dist(self.x, self.y, origin[0], origin[1])
            return d <= r_outer and d >= r_inner

        def __str__(self):
            return "({:.4f}, {:.4f}, {})".format(self.x, self.y, self.num_rings)

        def __repr__(self):
            return "Particle " + str(self)

    @staticmethod
    def dist(x1, y1, x2, y2):
        # distance between 2 points
        return numpy.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_position_from_landmarks(self, landmark_dists, estimated_position):
        """
        Estimate the position using relative distances to landmarks.
        :param landmark_dists: list of tuples containing the landmark's location and the distance measurement.
        :param estimated_position: (x, y) of the robot's estimated position
        :return: tuple, ((x, y), confidence) of estimated position
        """

        def plot(particles):  # for debugging
            from matplotlib import pyplot
            color = {0: 'red', 1: 'blue', 2: 'green', 3: 'yellow', 4: 'orange'}
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
            d = self.dist(estimated_position[0], estimated_position[1], origin[0], origin[1])
            a = origin[0] + v[0] / d * r, origin[1] + v[1] / d * r
            return a, self.CONFIDENCE_1LM

        # generate a list of particles
        particles = []
        for lmk in landmark_dists:
            particles += [self.Particle.from_ring(lmk[0], lmk[1], self.measurement_noise) for _ in
                          range(self.PARTICLE_BATCH_SIZE)]

        # record the number of rings each particle is in range in
        most_rings = 0  # keep track of the most number of rings particles fall in
        for lmk in landmark_dists:
            #           x, y, radius
            ring_outer = lmk[1] + self.measurement_noise
            ring_inner = lmk[1] - self.measurement_noise

            for particle in particles:
                if particle.is_inside_rings(lmk[0], ring_outer, ring_inner):
                    particle.num_rings += 1

                if particle.num_rings > most_rings:  # update most rings
                    most_rings = particle.num_rings

        if most_rings == 1:
            # no overlap between rings, error out
            return None, None

        best_particles = list(filter(lambda p: p.num_rings == most_rings, particles))
        # plot(best_particles)  # Uncomment for debug plot

        if most_rings < len(
                landmark_dists) or most_rings == 2:  # did not have an ideal reading/overlap zone (likely a bad distance measurement)
            # use estimated position to narrow particles
            best_particles = filter(
                lambda p: p.is_inside_rings(estimated_position, self.estimation_proximity_threshold, 0), best_particles)

        coords = list(map(lambda p: (p.x, p.y), best_particles))  # convert to tuples for numpy

        if len(coords) == 0:  # No particles were close to the estimated position!
            return None, None

        center = numpy.mean(list(coords), axis=0)
        return tuple(center), self.CONFIDENCE_2LM if most_rings == 2 else self.CONFIDENCE_3LM

    def get_orientation_from_landmarks(self, landmark_thetas, position):
        """
        Find the orientation from the detected landmarks and detected position
        :param landmark_thetas: list of tuples containing a landmark and the robot's orientation offset from the landmark
        :param position: (x,y) position returned from get_position_from_landmarks()
        :return: global orientation (theta) in radians
        """

        def get_theta_to_obj(landmark):
            """use position to determine the global rotation to see the object head on"""
            dy = position[1] - landmark.position[1]
            dx = position[0] - landmark.position[0]
            theta = math.atan(dy / dx)

            if dy >= 0:
                if dx >= 0:
                    return math.pi / 2 - theta  # Q1
                else:
                    return 3 * math.pi / 2 + theta  # Q2

            else:
                if dx >= 0:
                    return math.pi / 2 + theta  # Q4
                else:
                    return 3 * math.pi / 2 - theta  # Q3

        global_thetas = []
        for landmark, theta in landmark_thetas:
            # compute global thenta to landmark (observed theta looking North)
            landmark_global_theta = get_theta_to_obj(landmark)

            global_thetas.append(landmark_global_theta - theta)  # global - offset

        # remove outliers
        g_avg = float(numpy.mean(global_thetas))
        if len(landmark_thetas) > 2:
            g_std = float(numpy.std(global_thetas))
            for val in global_thetas:
                z = (val - g_avg) / g_std
                if abs(z) > 2:  # try to remove bad readings (false positive detections)
                    global_thetas.remove(val)

        # average
        return float(numpy.mean(global_thetas))