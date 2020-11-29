"""
    created by Jordan Gassaway, 10/27/2020
    WallE:
"""
import numpy
import time
import math

import jetbot
from PositionModel import PositionModel
from PositionDetector import PositionDetector

from threading import Thread, Event


class WallE:
    ERROR_THETA = math.radians(10)  # 10 deg
    ERROR_DISTANCE = 0.1    # 10 cm
    UPDATE_DT = 0.15        # 20 ms
    EVAL_POSITION = 0.05    # 50 ms

    def __init__(self):
        self.drive = self.DriveModel()
        self.drive.stop()   # Kill any previous drive commands
        self.position = PositionModel()
        self.movement = self.MovementModel()

        self.locator = PositionDetector()
        self.updateTimer = self.UpdateThread(self.UPDATE_DT, self.updatePosition)     # update position every 20 ms
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
        self.locator.logging = True
        while not self._is_at_position(x, y):
            # get angle to rotate towards x, y
            theta_drive = self.position.get_abs_angle_to(x, y)
            print("Turn to {:.2f}".format(theta_drive))
            # rotate towards x,y
            self._turn_to_theta(theta_drive)

            # drive forward to x, y
            self._drive_to_x_y(x, y)
            self.drive.stop()

        # rotate to theta
        self._turn_to_theta(theta)
        self.locator.logging = False

    def _drive_to_x_y(self, x, y):
        """
        Drive to an (x, y) point assuming a current velocity and angular velocity.
        Adjusts wheel speeds rather than explicitly turning/driving forward.
        """
        print("Drive to {:.2f} {:.2f}".format(x, y))

        while not self._is_at_position(x, y):
            # get relative angle to
            d_theta = self.position.get_rel_angle_to(self.position.get_abs_angle_to(x, y))

            # if off by less than 10 deg, drive forward
            if abs(d_theta) < math.radians(self.ERROR_THETA):
                self.drive.forward()
            elif d_theta > 0:   # if to the left, compute left arc
                pass
            else: # if right, compute right arc
                pass

            self.speed_r, self.speed_l, _ = self.movement.arc_to(x, y, self.position)
            self.drive.at_speed(self.speed_r, self.speed_l)

            time.sleep(self.EVAL_POSITION)  # reevaluate every .2 sec

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
        speed_r, speed_l = self.drive.get_current_speed()
        v, omega = self.movement.get_current_v_omega(speed_r, speed_l)
#         print("speed_r: {:.4f} speed_l: {:.4f}".format(speed_r, speed_l))
        new_pos = self.locator.get_position(v / 100.0, omega, self.UPDATE_DT)
        self.position.set_position(new_pos[0], new_pos[1], new_pos[2])

    def close(self):
        self.locator.close()
        self.updateTimer.cancel()
        self.updateTimer.join()

    def _is_oriented_towards(self, desired_theta):
        return abs(self.position.theta - desired_theta) < self.ERROR_THETA

    def _is_at_position(self, x, y):
        return self.position.get_distance_to(x, y) < self.ERROR_DISTANCE

    def calibrate(self):
        self.drive.calibrate()
        self.movement.calibrate(self.drive)
        self.locator.calibrate()

    def _turn_to_theta(self, theta):
        """turn to absolute orientation theta"""
        print("Turn to {:.2f}".format(theta))
        delta = self.position.get_rel_angle_to(theta)
        if delta > 0:
            self.drive.left()
        else:
            self.drive.right()
        while delta > self.ERROR_THETA:
            time.sleep(self.EVAL_POSITION)
            delta = self.position.get_rel_angle_to(theta)

        self.drive.stop()

    class UpdateThread(Thread):
        def __init__(self, interval, update_func):
            super().__init__()
            self.update_func = update_func
            self.interval = interval
            self._cancel_flag = Event()

        def cancel(self):
            self._cancel_flag.set()

        def run(self):
            while not self._cancel_flag.is_set():
                ts = time.time()
                self.update_func()
                tf = time.time()
                t_exec = tf - ts
                if t_exec < self.interval:
                    time.sleep(self.interval - t_exec)
                else:
                    print("Timer over run! Exec Time: {:.5f}".format(t_exec))

    class DriveModel():
        """Abstraction for driving the robot. Converts velocities to power settings."""
        BASE_POWER = 0.5
        R_L_OFFSET = 0.035
        SPEED_PWR_RATIO = 0.0209
        BASE_SPEED = BASE_POWER / SPEED_PWR_RATIO

        def __init__(self):
            self.robot = jetbot.robot.Robot()
            self.speed_l = 0.0
            self.speed_r = 0.0

        def calibrate(self):
            # calibrate left and right speeds
            print("Alignment calibration")
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
            print("Alignment calibration complete: right/left offset: {}".format(self.R_L_OFFSET))
            input('Press any key to continue')

            print("Power calibration")
            lmb = []   # lambda (speed to power ratio)
            for pwr in [0.4, 0.44, 0.48, 0.52, 0.56, 0.58, 0.62]:
                # drive full speed for 1s
                self._set_motors(pwr, pwr)
                time.sleep(1)
                self.robot.stop()
                # enter distance traveled
                ratio = pwr / float(input("Enter cm traveled: "))
                lmb.append(ratio)
                print("\t pwr to distance ratio: {}".format(ratio))

            self.SPEED_PWR_RATIO = numpy.mean(lmb, axis=0)
            self.BASE_SPEED = self.BASE_POWER / self.SPEED_PWR_RATIO
            print("Power calibration complete: pwr conversion factor: {}".format(self.SPEED_PWR_RATIO))

        def _set_motors(self, pwr_r, pwr_l):
            self.robot.set_motors(pwr_l, pwr_r + self.R_L_OFFSET)

        def _set_speed(self, speed_r, speed_l):
#             print("set speed r {} l {}".format(speed_r, speed_l))
            self.speed_l = speed_l
            self.speed_r = speed_r
            r_offset = self.R_L_OFFSET if speed_l >= 0 else -self.R_L_OFFSET
            self._set_motors(speed_r * self.SPEED_PWR_RATIO + r_offset, speed_l * self.SPEED_PWR_RATIO)

        def at_speed(self, speed_r, speed_l):
            """
            Set the rotation speeds of the right and left wheels
            :param speed_r: Right Wheel speed (cm/s)
            :param speed_l: Left Wheel speed (cm/s)
            """
            print("--drive: at speed {} {}".format(speed_r, speed_l))
            self._set_speed(speed_r, speed_l)

        def forward(self):
            print("--drive: forward")
            self._set_speed(self.BASE_SPEED, self.BASE_SPEED)

        def right(self):
            print("--drive: right")
            self._set_speed(self.BASE_SPEED, -self.BASE_SPEED)

        def left(self):
            print("--drive: left")
            self._set_speed(-self.BASE_SPEED, self.BASE_SPEED)

        def stop(self):
            print("--drive: stop")
            self.speed_l = 0.0
            self.speed_r = 0.0
            self.robot.stop()

        def get_current_speed(self):
            return self.speed_r, self.speed_l

    class MovementModel():
        """Model path planning and movement"""
        # WHEEL_CIRCUMFERENCE = 0.215  # cm
        WHEEL_SEPARATION = 13.1 # W (cm)
        MAX_SPEED = 12.0    # maximum wheel speed in cm/s
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
            :return: (speed_r, speed_l, arc_theta) in cm/s, radians
            """
            sec_len = position.get_distance_to(x, y) * 100
            turn_theta = position.get_rel_angle_to(position.get_abs_angle_to(x, y), allow_clockwise=True)
            left_turn = turn_theta > 0

            if abs(turn_theta) > self.RAD_90: # just turn sharply instead of computing an arc.
                v_outer, v_inner = self.MAX_SPEED, 1.0
                return (v_outer, v_inner, math.pi) if left_turn else (v_inner, v_outer, -math.pi)

            sec_theta = self.RAD_90 - abs(turn_theta)
            arc_theta = math.pi - 2* sec_theta
            radius = sec_len / (2* math.cos(sec_theta))

            w = self.WHEEL_SEPARATION
            d_outer = (radius + w/2) * arc_theta
            d_inner = (radius - w/2) * arc_theta
            dt = (d_outer / self.MAX_SPEED)  # time to travers the arc

            v_r, v_l = (d_outer/dt, d_inner/dt) if left_turn else (d_inner/dt, d_outer/dt)
            return v_r, v_l, arc_theta

        def _get_turn_radius(self, vo, vi):
            """:returns turn radius from vr and vl in cm"""
            v_ratio = vo / vi
            w = self.WHEEL_SEPARATION / 2
            r = (w * (v_ratio + 1) / (v_ratio - 1))
            return r

        def _get_velocity(self, vi, r):
            """returns forward velocity of the robot (irrespective of orientation)"""
            return r * vi / (r - self.WHEEL_SEPARATION/2)

        def get_current_v_omega(self, vr, vl):
            """get forward velocity and angular velocity from wheel velocities
            :returns forward velocity (cm/s), angular velocity (rad/s)
            """
            if vr == vl:    # if speeds are the same, we are going straight (not turning)
                return vr, 0.0
            elif vr == -vl: # special case, rotating in place. r = w/2, v = vr
                return 0.0, vr / (self.WHEEL_SEPARATION / 2) * 0.7     # Seems to rotate slower than predicted wheel velocities

            vo, vi = (vr, vl) if vr > vl else (vl, vr)
            r = self._get_turn_radius(vo, vi)
            v = self._get_velocity(vi, r)

            return v, v / r