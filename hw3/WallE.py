"""
    created by Jordan Gassaway, 10/27/2020
    WallE:
"""
import time
import math

import jetbot
from hw3.PositionModel import PositionModel
from hw3.PositionDetector import PositionDetector

from threading import Timer


class WallE:
    SPEED_LEFT = 0.5
    SPEED_RIGHT = 0.515

    ERROR_THETA = math.radians(10)  # 10 deg
    ERROR_DISTANCE = 0.1    # 10 cm

    def __init__(self):
        self.robot = jetbot.robot.Robot()
        self.position = PositionModel()
        self.movement = self.MovementModel()
        self.speed_right = 0.0
        self.speed_left = 0.0

        self.locator = PositionDetector()
        self.updateTimer = Timer(0.02, self.updatePosition)     # update position every 20 ms
        self.updateTimer.start()


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
        while not self._is_at_position(x, y):
            # get angle to rotate towards x, y
            theta_drive = self.position.get_abs_angle_to(x, y)
            while not self._is_oriented_towards(theta_drive):
                # rotate towards x,y
                self._turn_to_theta(theta_drive)

            # get distance to x, y
            dist = self.position.get_distance_to(x, y)
            # drive forward to x, y
            self._drive_distance(dist)

        # rotate to theta
        while not self._is_oriented_towards(theta):
            self._turn_to_theta(theta)

    def _drive_to_continuous(self, x, y):
        """
        Drive to an (x, y) point assuming a current velocity and angular velocity.
        Adjusts wheel speeds rather than explicitly turning/driving forward.
        """
        while not self._is_at_position(x, y):
            # TODO
            time.sleep(0.2)  # reevaluate every .2 sec

    def drive_path(self, waypoints):
        """
        Drive a path passing through each of the waypoints
        :param waypoints list of (x, y) coordinates
        """
        # turn towards first point and start driving
        self._turn_to_theta(self.position.get_abs_angle_to(*waypoints[0]))
        self._set_speed(self.SPEED_RIGHT, self.SPEED_LEFT)

        for point in waypoints:
            self._drive_to_continuous(*point)

        self.robot.stop()

    def updatePosition(self):
        new_pos = self.locator.get_position(self.speed_left, self.speed_right)
        self.position.set_position(**new_pos)

    def close(self):
        self.locator.close()
        self.updateTimer.cancel()

    def _is_oriented_towards(self, desired_theta):
        return abs(self.position.theta - desired_theta) < self.ERROR_THETA

    def _is_at_position(self, x, y):
        return self.position.get_distance_to(x, y) < self.ERROR_DISTANCE

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

        self._right()
        time.sleep(t_rotate)
        self.robot.stop()

    def _drive_distance(self, distance):
        """drive distance forward"""
        t_drive = self.movement.get_drive_duration(distance)

        self._forward()
        time.sleep(t_drive)
        self.robot.stop()

    def _set_speed(self, speed_r, speed_l):
        self.speed_right = speed_r
        self.speed_left = speed_l
        self._drive()

    def _drive(self):
        self.robot.set_motors(self.speed_left, self.speed_right)

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
