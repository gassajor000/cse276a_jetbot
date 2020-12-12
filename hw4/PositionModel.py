"""
    created by Jordan Gassaway, 11/10/2020
    PositionModel: 
"""
import math


class PositionModel():
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    # movement update functions
    def move_forward(self, d):
        """update position after driving forward d meters"""
        self.x += d * math.sin(self.theta)
        self.y += d * math.cos(self.theta)

    def rotate_clockwise(self, theta):
        """update position after rotating clockwise theta radians"""
        self.theta = (self.theta + theta) % (2 * math.pi)

    def get_position(self):
        """:returns current position and orientation"""
        return self.x, self.y, self.theta

    def set_position(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def get_distance_to(self, x, y):
        """returns the distance from the current position to the specified point (assumes meters)"""
        return math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)

    def get_abs_angle_to(self, x, y):
        """returns absolute angle between vertical at current position and x, y"""
        if x >= self.x:
            if y >= self.y:
                quadrant = 1
            else:
                quadrant = 4
        else:
            if y >= self.y:
                quadrant = 2
            else:
                quadrant = 3

        if self.x == x:  # handle vertical case
            return 0 if quadrant == 1 else math.pi

        theta_horizontal = abs(math.atan((y - self.y) / (x - self.x)))

        adjustment = {1: lambda t: 3*math.pi / 2 + t,  # 270 deg + theta
                      2: lambda t: math.pi / 2 - t,  # 90 deg - theta
                      3: lambda t: math.pi / 2 + t,  # 90 deg + theta
                      4: lambda t: 3*math.pi / 2 - t}  # 270 deg - theta

        angle = adjustment[quadrant](theta_horizontal)
        # print('abs angle {:.4f}'.format(angle))
        return angle

    def get_rel_angle_to(self, theta, allow_clockwise=False):
        """returns counter clockwise offset between theta and current orientation"""
        # t1 >= t0: t1 - t0, t0 > t1: (t0 - 2pi) + t1
        rel_angle =  theta - self.theta if theta >= self.theta else (2 * math.pi) - self.theta + theta

        if allow_clockwise and rel_angle > math.pi:     # subtract 2pi to make it a negitve angle (clockwise turn) instead
            rel_angle -= 2 * math.pi

        # print('rel angle {:.4f}'.format(rel_angle))
        return rel_angle