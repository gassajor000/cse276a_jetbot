"""
    created by Jordan Gassaway, 10/27/2020
    WallE:
"""
import numpy
import time
import math

import jetbot
from hw3.PositionModel import PositionModel
from hw3.PositionDetector import PositionDetector

from threading import Timer


class WallE:
    ERROR_THETA = math.radians(10)  # 10 deg
    ERROR_DISTANCE = 0.1    # 10 cm

    def __init__(self):
        self.drive = self.DriveModel()
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

            # drive forward to x, y
            self._drive_to_x_y(x, y)

        # rotate to theta
        while not self._is_oriented_towards(theta):
            self._turn_to_theta(theta)

    def _drive_to_x_y(self, x, y):
        """
        Drive to an (x, y) point assuming a current velocity and angular velocity.
        Adjusts wheel speeds rather than explicitly turning/driving forward.
        """
        while not self._is_at_position(x, y):
            # get relative angle to
            d_theta = self.position.get_rel_angle_to(self.position.get_abs_angle_to(x, y))

            # if off by less than 10 deg, drive forward
            if abs(d_theta) < math.radians(self.ERROR_THETA):
                self.drive.forward()
            elif d_theta > 0:   # if to the left, compute left arc

                self.drive.at_speed(*self.movement.diff_speeds_to())
            else: # if right, compute right arc
                self.drive.at_speed(*self.movement.diff_speeds_to())

            time.sleep(0.2)  # reevaluate every .2 sec

    def drive_path(self, waypoints):
        """
        Drive a path passing through each of the waypoints
        :param waypoints list of (x, y) coordinates
        """
        # turn towards first point and start driving
        self._turn_to_theta(self.position.get_abs_angle_to(*waypoints[0]))
        self.drive.forward()

        for point in waypoints:
            self._drive_to_x_y(*point)

        self.drive.stop()

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
        self.drive.calibrate()
        self.movement.calibrate()
        self.locator.calibrate()

    def _turn_to_theta(self, theta):
        """turn to absolute orientation theta"""
        delta = self.position.get_rel_angle_to(theta)
        while delta > self.ERROR_THETA:
            if delta > 0:
                self.drive.left()
            else:
                self.drive.right()
            time.sleep(0.2)
            delta = self.position.get_rel_angle_to(theta)

        self.drive.stop()

    class DriveModel():
        """Abstraction for driving the robot. Converts velocities to power settings."""
        BASE_POWER = 0.5
        R_L_OFFSET = 0.015
        SPEED_PWR_RATIO = 5

        def __init__(self):
            self.robot = jetbot.robot.Robot()

        def at_speed(self, speed_r, speed_l):
            """
            Set the rotation speeds of the right and left wheels
            :param speed_r: Right Wheel speed (cm/s)
            :param speed_l: Left Wheel speed (cm/s)
            """
            pwr_r = speed_r * self.SPEED_PWR_RATIO
            pwr_l = speed_l * self.SPEED_PWR_RATIO
            self._set_motors(pwr_l, pwr_r)

        def calibrate(self):
            # calibrate left and right speeds
            print("Adjust right speed until robot drives straight")
            cmd = 'x'
            while cmd not in ['c', 'C']:
                self.forward()
                cmd = input('Type L to curve left, R to curve right, or C to continue')
                if cmd in ['l', 'L']:
                    self.R_L_OFFSET += 0.005
                if cmd in ['r', 'R']:
                    self.R_L_OFFSET -= 0.005

            self.robot.stop()
            print("Speed calibration complete: right/left offset: {}".format(self.R_L_OFFSET))
            input('Press any key to continue')

            print("Distance calibration")
            lmb = []   # lambda (speed to power ratio)
            for pwr in [0.25, 0.5, 0.75]:
                # drive full speed for 1s
                self._set_motors(pwr, pwr)
                time.sleep(1)
                self.robot.stop()
                # enter distance traveled
                lmb.append(pwr / float(input("Enter cm traveled: ")))

            self.SPEED_PWR_RATIO = numpy.mean(lmb, axis=0)

            print("Rotation calibration")
            # rotate full speed for 1s
            self.right()
            time.sleep(1)
            self.robot.stop()
            # enter angle rotated
            t = float(input("Enter degrees rotated (clockwise): "))

        def _set_motors(self, pwr_r, pwr_l):
            self.robot.set_motors(pwr_l, pwr_r + self.R_L_OFFSET)

        def forward(self):
            self.robot.set_motors(self.BASE_POWER, self.BASE_POWER + self.R_L_OFFSET)

        def right(self):
            self.robot.set_motors(self.BASE_POWER, -(self.BASE_POWER + self.R_L_OFFSET))

        def left(self):
            self.robot.set_motors(-self.BASE_POWER, self.BASE_POWER + self.R_L_OFFSET)

        def stop(self):
            self.robot.stop()

    class MovementModel():
        """Model path planning and movement"""
        # WHEEL_CIRCUMFERENCE = 0.215  # cm
        WHEEL_SEPARATION = 13.1 # W (cm)
        MAX_SPEED = 50.0    # maximum wheel speed in cm/s
        RAD_90 = math.pi / 2

        def calibrate(self, drive_model):
            print("Speed calibration")
            input('Press any key to continue')
            # drive full speed for 1s
            drive_model.forward()
            time.sleep(1)
            drive_model.stop()
            # enter distance traveled
            dist = float(input("Enter cm traveled: "))
            self.MAX_SPEED = dist

        def arc_to(self, x, y, position: PositionModel):
            """
            Compute an arc to x, y from current position. Caps the speed of any wheel at MAX_SPEED
            :return: (speed_r, speed_l) in cm/s
            """
            sec_len = position.get_distance_to(x, y) * 100
            turn_theta = position.get_rel_angle_to(position.get_abs_angle_to(x, y))
            left_turn = turn_theta > 0

            if abs(turn_theta) > self.RAD_90: # just turn sharply instead of computing an arc.
                v_outer, v_inner = self.MAX_SPEED, 1.0
                return v_outer, v_inner if left_turn else v_inner, v_outer

            sec_theta = self.RAD_90 - abs(turn_theta)
            arc_theta = math.pi - 2* sec_theta
            radius = sec_len / (2* math.cos(sec_theta))

            w = self.WHEEL_SEPARATION
            d_outer = (radius + w/2) * arc_theta
            d_inner = (radius - w/2) * arc_theta
            dt = (d_outer * arc_theta / self.MAX_SPEED)  # time to travers the arc

            v_r, v_l = (d_outer/dt, d_inner/ dt) if left_turn else (d_inner / dt, d_outer/ dt)
            return v_r, v_l
