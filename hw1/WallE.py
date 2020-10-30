"""
    created by Jordan Gassaway, 10/27/2020
    WallE:
"""
import time
import math

import jetbot


class WallE:
    SPEED = 0.5
    def __init__(self):
        self.robot = jetbot.robot.Robot()
        self.position = self.PositionModel()
        self.movement = self.MovementModel()

    def drive_to(self, x, y, theta):
        # get angle to rotate towards x, y
        # rotate towards x,y

        # get distance to x, y
        # drive forward to x, y

        # get angle to theta
        # rotate to theta
        pass

    def calibrate(self):
        # drive full speed for 1s
        self.robot.forward(self.SPEED)
        time.sleep(1)
        self.robot.stop()
        # enter distance traveled
        d = float(input("Enter cm traveled: "))

        # rotate full speed for 1s
        self.robot.right(self.SPEED)
        time.sleep(1)
        self.robot.stop()
        # enter angle rotated
        t = float(input("Enter degrees rotated (clockwise): "))

        # update movement model
        self.movement.calibrate(d/100.0, math.radians(t))

    def _turn_to_theta(self, theta):
        """turn to absolute orientation theta"""
        pass

    def _drive(self, distance):
        """drive distance forward"""
        self.robot.forward()

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
            return (self.x, self.y, self.theta)

        def get_distance_to(self, x, y):
            """returns the distance from the current position to the specified point"""
            return math.sqrt((x - self.x)**2 + (y - self.y)**2)

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

            if self.x == x:     # handle vertical case
                return 0 if quadrant == 1 else math.pi

            theta_horizontal = abs(math.atan((y - self.y)/ (x - self.x)))

            adjustment = {1: lambda t: math.pi/2 - t,   # 90 deg - theta
                          2: lambda t: 3*math.pi/2 + t, # 270 deg + theta
                          3: lambda t: 3*math.pi/2 - t,    # 270 deg - theta
                          4: lambda t: math.pi/2 + t }    # 90 deg + theta

            return adjustment[quadrant](theta_horizontal)

        def get_rel_angle_to(self, theta):
            """returns clockwise offset between theta and current orientation"""
            # t1 >= t0: t1 - t0, t0 > t1: (t0 - 2pi) + t1
            return theta - self.theta if theta >= self.theta else (2 * math.pi) - self.theta + theta


    class MovementModel():
        # WHEEL_CIRCUMFERENCE = 21.5  # cm
        # WHEEL_SEPARATION = 10.2 # cm

        def __init__(self):
            self.distance_ref = .265    # meters traveled over 1s
            self.angle_ref = 2.967      # radians (cw) rotated over 1s

        def calibrate(self, distance, angle):
            self.distance_ref = distance
            self.angle_ref = angle

        def get_drive_duration(self, distance, speed=0.5):
            """returns time needed to drive :distance: at :speed:"""

        def get_rotation_time(self, theta):
            """returns time needed to rotate clockwise theta radians"""
