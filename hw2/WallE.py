"""
    created by Jordan Gassaway, 10/27/2020
    WallE:
"""
import time
import math

import jetbot
from .PositionModel import PositionModel
from .PositionDetector import PositionDetector


class WallE:
    SPEED_LEFT = 0.5
    SPEED_RIGHT = 0.515

    def __init__(self):
        self.robot = jetbot.robot.Robot()
        self.position = PositionModel()
        self.movement = self.MovementModel()
        self.locator = PositionDetector()

    def drive_to(self, x, y, theta):
        """
        while not @ x,y:
            while not theta_to_xy:
                turn to x,y
                evaluate position.theta

            while distance_traveled != dist:
                drive fwd
                evaluate x,y
                evaluate distance_traveled

        while not @ theta:
            turn to theta
            evaluate position.theta
        """


        # get angle to rotate towards x, y
        theta_drive = self.position.get_abs_angle_to(x, y)
        # rotate towards x,y
        self._turn_to_theta(theta_drive)

        # get distance to x, y
        dist = self.position.get_distance_to(x, y)
        # drive forward to x, y
        self._drive(dist)

        # rotate to theta
        self._turn_to_theta(theta)

    def calibrate(self):
        # calibrate left and right speeds
        print("Adjust right speed until robot drives straight")
        cmd = 'x'
        while cmd not in ['c', 'C']:
            self._forward()
            cmd = input('Type L to curve left, R to curve right, or C to continue')
            if cmd in ['l', 'L']:
                self.SPEED_RIGHT += 0.005
            if cmd in ['r', 'R']:
                self.SPEED_RIGHT -= 0.005

        self.robot.stop()
        print("Speed calibration complete: left speed {:.2f} right speed {:.2f}".format(self.SPEED_LEFT, self.SPEED_RIGHT))
        input('Press any key to continue')
        print("Distance calibration")
        # drive full speed for 1s
        self._forward()
        time.sleep(1)
        self.robot.stop()
        # enter distance traveled
        d = float(input("Enter cm traveled: "))

        print("Rotation calibration")
        # rotate full speed for 1s
        self._right()
        time.sleep(1)
        self.robot.stop()
        # enter angle rotated
        t = float(input("Enter degrees rotated (clockwise): "))

        # update movement model
        self.movement.calibrate(d/100.0, math.radians(t))

        self.locator.calibrate()

    def _turn_to_theta(self, theta):
        """turn to absolute orientation theta"""
        delta = self.position.get_rel_angle_to(theta)
        t_rotate = self.movement.get_rotation_time(delta)
        new_pos = self.locator.get_position(0.0, delta)
        self.position.set_position(**new_pos)

        self._right()
        time.sleep(t_rotate)
        self.robot.stop()

    def _drive(self, distance):
        """drive distance forward"""
        t_drive = self.movement.get_drive_duration(distance)
        new_pos = self.locator.get_position(distance, 0.0)
        self.position.set_position(**new_pos)

        self._forward()
        time.sleep(t_drive)
        self.robot.stop()

    def _forward(self):
        self.robot.set_motors(self.SPEED_LEFT, self.SPEED_RIGHT)

    def _right(self):
        self.robot.set_motors(self.SPEED_LEFT, -self.SPEED_RIGHT)


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
            return distance / self.distance_ref

        def get_rotation_time(self, theta, speed=0.5):
            """returns time needed to rotate clockwise theta radians"""
            return theta / self.angle_ref
