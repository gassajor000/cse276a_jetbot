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
        d = float(input("Enter distance traveled: "))

        # rotate full speed for 1s
        self.robot.right(self.SPEED)
        time.sleep(1)
        self.robot.stop()
        # enter angle rotated
        t = float(input("Enter degrees rotated (clockwise): "))

        # update movement model
        self.movement.calibrate(d, math.radians(t))

    def _turn_to_theta(self, theta):
        """turn to absolute orientation theta"""
        pass

    def _drive(self, distance):
        """drive distance forward"""
        self.robot.forward()

    class PositionModel():
        def __init__(self):
            self.x = 0
            self.y = 0
            self.theta = 0

        # movement update functions
        def move_forward(self, d):
            """update position after driving forward d meters"""

        def rotate_clockwise(self, theta):
            """update position after rotating clockwise theta radians"""

        def get_distance_to(self, x, y):
            """returns the distance from the current position to the specified point"""

        def get_abs_angle_to(self, x, y):
            """returns absolute angle between vertical at current position and x, y"""

        def get_rel_angle_to(self, theta):
            """returns clockwise offset between theta and current orientation"""

    class MovementModel():
        # WHEEL_CIRCUMFERENCE = 21.5  # cm
        # WHEEL_SEPARATION = 10.2 # cm

        def __init__(self):
            self.distance_ref = 1   # meters traveled over 1s
            self.angle_ref = 3.14   # angle (cw) rotated over 1s

        def calibrate(self, distance, angle):
            self.distance_ref = distance
            self.angle_ref = angle

        def get_drive_duration(self, distance, speed=0.5):
            """returns time needed to drive :distance: at :speed:"""

        def get_rotation_time(self, theta):
            """returns time needed to rotate clockwise theta radians"""
