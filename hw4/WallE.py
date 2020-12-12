"""
    created by Jordan Gassaway, 10/27/2020
    WallE:
"""
import json
from typing import List

import numpy
import time
import math

import jetbot
from PositionModel import PositionModel
from PositionDetector import PositionDetector

from threading import Thread, Event

from LandmarkDetector import Landmark
from PathPlanner import Point
from PathPlanner.PathPlanner import PathPlanner
from QRDetector import QRDetector


class WallE:
    ERROR_THETA = math.radians(10)  # 10 deg
    ERROR_DISTANCE = 0.05    # 05 cm
    UPDATE_DT = 0.2        # 20 ms
    EVAL_POSITION = 0.05    # 50 ms

    def __init__(self, map_file, init_pos=(0.0, 0.0, 0.0)):
        self.drive = self.DriveModel()
        self.drive.stop()   # Kill any previous drive commands
        self.position = PositionModel()
        self.movement = self.MovementModel()

        boundaries, obstacles, landmarks = self._read_map(map_file)
        self.planner = PathPlanner(boundaries, obstacles)

        self.locator = PositionDetector(QRDetector(landmarks), init_pos=init_pos)
        self.updateTimer = self.UpdateThread(self.EVAL_POSITION, self.updatePosition)     # update position every 20 ms
        self.time_since_sensor_read = 0.0
        self.last_update = time.time()
        self.updateTimer.start()

    @staticmethod
    def _read_map(map_file) -> (List[Point], List[List[Point]], List[Landmark]):
        with open(map_file) as f:
            map_data = json.load(f)
            boundaries = list(map(lambda p: Point(p[0], p[1]) , map_data['bounds']))
            obstacles = [list(map(lambda p: Point(p[0], p[1]) , obstacle)) for obstacle in map_data['obstacles']]
            landmarks = []
            for landmark in map_data['landmarks']:
                x, y = landmark['position'][0], landmark['position'][1]
                label = bytes(landmark['data'], 'utf-8') if landmark['category'] == 'QR code' else landmark['label']
                landmarks.append(Landmark((x, y), landmark['height'], landmark['name'], landmark['category'], label))

            return boundaries, obstacles, landmarks

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

    def _drive_to_x_y(self, x, y, r=None):
        """
        Drive to an (x, y) point assuming a current velocity and angular velocity.
        Adjusts wheel speeds rather than explicitly turning/driving forward.
        """
        print("Drive to {:.2f} {:.2f}".format(x, y))
        self.locator.logging = True

        while not self._is_at_position(x, y):
            desired_theta = self.position.get_abs_angle_to(x, y)
            d_theta = self.position.get_rel_angle_to(desired_theta, allow_clockwise=True)
            # if off by less than 10 deg, drive forward
            if abs(d_theta) < self.ERROR_THETA:
                self.drive.forward()
            else:    # if to the left, compute left arc
                self._turn_to_theta(desired_theta)

            time.sleep(self.EVAL_POSITION)  # reevaluate every .2 sec

        self.locator.logging = False

    def drive_path(self, waypoints):
        """
        Drive a path passing through each of the waypoints
        :param waypoints list of (x, y) coordinates
        """
        # turn towards first point and start driving
        # self._turn_to_theta(self.position.get_abs_angle_to(*waypoints[0]))
        # self.drive.forward()

        for point in waypoints:
            self._drive_to_x_y(*point)

        self.drive.stop()

    def navigate_to(self, x, y):
        """
        Compute and drive a path to x, y location.
        :param x: x coordinate
        :param y: y coordinate
        """
        start, end = Point(self.position.x, self.position.y), Point(x, y)
        path = self.planner.getPath(start, end)
        self.drive_path(path)

    def updatePosition(self):
        dt = time.time() - self.last_update
        self.last_update = time.time()
        self.time_since_sensor_read += dt
        read_sensors = self.time_since_sensor_read > self.UPDATE_DT

#         print("speed_r: {:.4f} speed_l: {:.4f}".format(speed_r, speed_l))
        velocity, omega = self.locator.get_v_omega()
        accel, alpha = self.drive.get_acceleration(velocity), self.drive.get_alpha(omega)
        new_pos = self.locator.get_position(accel, omega, dt, read_sensors)
        self.position.set_position(new_pos[0], new_pos[1], new_pos[2])

        if read_sensors:
            self.time_since_sensor_read = 0.0

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
        delta = self.position.get_rel_angle_to(theta, allow_clockwise=True)

        while abs(delta) > self.ERROR_THETA:
            if delta > 0:
                self.drive.left()
            else:
                self.drive.right()
            print("delta {:.4f}".format(delta))
            time.sleep(self.EVAL_POSITION)
            delta = self.position.get_rel_angle_to(theta, allow_clockwise=True)

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
                elif t_exec > WallE.UPDATE_DT:
                    print("Timer over run! Exec Time: {:.5f}".format(t_exec))

    class DriveModel():
        """Abstraction for driving the robot. Converts velocities to power settings."""
        BASE_POWER = 0.4
        R_L_OFFSET = 0.012
        SPEED_PWR_RATIO = 0.019
        BASE_SPEED = BASE_POWER / SPEED_PWR_RATIO
        ALPHA_INCREMENTS = { 1.396: 5.585, 1.99: 0.792 }
        ACCEL_INCREMENTS = {0.16: 0.32, 0.22: 0.08 }
        DECEL = -0.32
        WINDDOWN = 5.585

        def __init__(self):
            self.robot = jetbot.robot.Robot()
            self.state = 'stop'

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

            time_increments = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0]
            dt = 0.25

            print("Alpha calibration")
            alpha_steps = {}
            omega_last = 0.0
            theta_last = 0.0
            for d_time in time_increments:
                self.left()
                time.sleep(d_time)
                self.stop()

                theta = math.radians(float(input("Input degrees rotated counter clockwise")))
                omega = (theta -  theta_last) / dt
                alpha = (omega - omega_last) / dt
                alpha_steps[omega_last] = alpha
                print("alpha {:.3f} omega {:.3f} theta {:.3f}".format(alpha, omega, theta))

                theta_last = theta
                omega_last = omega

            print("Alpha steps: {}".format(alpha_steps))
            self.ALPHA_INCREMENTS = alpha_steps

            print("Acceleration calibration")
            accel_steps = {}
            v_last = 0.0
            d_last = 0.0
            for d_time in time_increments:
                self.forward()
                time.sleep(d_time)
                self.stop()

                d = float(input("Input cm traveled"))
                v = (d -  d_last) / dt
                a = (v - v_last) / dt
                accel_steps[v_last] = a
                print("a {:.3f} v {:.3f} d {:.3f}".format(a, v, d))

                v_last = v
                d_last = d

            print("Acceleration steps: {}".format(accel_steps))
            self.ACCEL_INCREMENTS = accel_steps

        def _set_motors(self, pwr_r, pwr_l):
            self.robot.set_motors(pwr_l, pwr_r + self.R_L_OFFSET)

        def _set_speed(self, speed_r, speed_l):
            # print("set speed r {} l {}".format(speed_r, speed_l))
            r_offset = self.R_L_OFFSET if speed_l >= 0 else -self.R_L_OFFSET
            self._set_motors(speed_r * self.SPEED_PWR_RATIO + r_offset, speed_l * self.SPEED_PWR_RATIO)

        def at_speed(self, speed_r, speed_l):
            """
            Set the rotation speeds of the right and left wheels
            :param speed_r: Right Wheel speed (cm/s)
            :param speed_l: Left Wheel speed (cm/s)
            """
            # print("--drive: at speed {} {}".format(speed_r, speed_l))
            # self._set_speed(speed_r, speed_l)
            raise RuntimeError('at_speed is no longer supported!')

        def forward(self):
            # print("--drive: forward")
            self._set_speed(self.BASE_SPEED, self.BASE_SPEED)
            self.state = 'forward'

        def right(self):
            # print("--drive: right")
            self._set_speed(-self.BASE_SPEED, self.BASE_SPEED)
            self.state = 'right'

        def left(self):
            # print("--drive: left")
            self._set_speed(self.BASE_SPEED, -self.BASE_SPEED)
            self.state = 'left'

        def stop(self):
            # print("--drive: stop")
            self.robot.stop()
            self.state = 'stop'

        def get_acceleration(self, velocity):
            abs_v = abs(velocity)

            if self.state == 'stop':
                return self.DECEL if velocity > 0.02 else 0.0
            elif self.state != 'forward':
                return 0.0

            for a_step in self.ACCEL_INCREMENTS:
                if abs_v < a_step:
                    return self.ACCEL_INCREMENTS[a_step]

            else:
                return 0.0      # saturate velocity

        def get_alpha(self, omega):
            abs_omega = abs(omega)
            sign = 1 if omega > 0.0 else -1

            if self.state == 'stop':
                if abs_omega < 0.04:
                    return 0.0
                if abs_omega < 0.1:
                    return (-1 * sign) * 0.1
                else:
                    return (-1 * sign) * self.WINDDOWN

            elif self.state == 'forward':
                return 0.0

            for omega_step in self.ALPHA_INCREMENTS:
                if abs_omega < omega_step:
                    return self.ALPHA_INCREMENTS[omega_step] * sign

            else:
                return 0.0  # saturate omega

    class MovementModel():
        """Model path planning and movement"""
        # WHEEL_CIRCUMFERENCE = 0.215  # cm
        WHEEL_SEPARATION = 13.1 # W (cm)
        MIN_SPEED = 12.0    # minimum wheel speed in cm/s
        MAX_SPEED = 24.0    # maximum wheel speed in cm/s
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
            self.MIN_SPEED = dist

        def arc_to(self, x, y, position: PositionModel):
            """
            Compute an arc to x, y from current position. Caps the speed of any wheel at MAX_SPEED
            :return: (speed_r, speed_l, arc_theta) in cm/s, radians
            """
            sec_len = position.get_distance_to(x, y) * 100
            turn_theta = position.get_rel_angle_to(position.get_abs_angle_to(x, y), allow_clockwise=True)
            left_turn = turn_theta > 0

            sec_theta = self.RAD_90 - abs(turn_theta)
            arc_theta = math.pi - 2* sec_theta
            radius = sec_len / (2* math.cos(sec_theta))
            w = self.WHEEL_SEPARATION

            if radius < w/2: # just turn sharply instead of computing an arc.
                v_outer, v_inner = self.MIN_SPEED, 0.5
#                 print('r {:.3f}, turn_theta {:.3f}'.format(radius, turn_theta))
#                 print('turn ' + ('left' if left_turn else 'right'))
                return (v_outer, v_inner, math.pi) if left_turn else (v_inner, v_outer, -math.pi)

            if abs(turn_theta) > self.RAD_90: # just turn sharply instead of computing an arc.
                v_outer, v_inner = self.MAX_SPEED, (self.MIN_SPEED - 2)
#                 print('r {:.3f}, turn_theta {:.3f}'.format(radius, turn_theta))
#                 print('turn ' + ('left' if left_turn else 'right'))
                return (v_outer, v_inner, math.pi) if left_turn else (v_inner, v_outer, -math.pi)

            d_outer = (radius + w/2) * arc_theta
            d_inner = (radius - w/2) * arc_theta
            dt = (d_inner / self.MIN_SPEED)  # time to travers the arc

            v_r, v_l = (d_outer/dt, d_inner/dt) if left_turn else (d_inner/dt, d_outer/dt)
            # print('arc r={} theta={}'.format(radius, turn_theta))
            return v_r, v_l, arc_theta

        def _get_turn_radius(self, vo, vi):
            """:returns turn radius from vr and vl in cm"""
            v_ratio = vo / vi
            w = self.WHEEL_SEPARATION / 2
            r = (w * (v_ratio + 1) / (v_ratio - 1))
            return r

#         def _get_velocity(self, vi, r):
#             """returns forward velocity of the robot (irrespective of orientation)"""
#             return r * vi / (r - self.WHEEL_SEPARATION/2)
#
#         def get_current_v_omega(self, vr, vl):
#             """get forward velocity and angular velocity from wheel velocities
#             :returns forward velocity (cm/s), angular velocity (rad/s)
#             """
#             if vr == vl:    # if speeds are the same, we are going straight (not turning)
#                 return vr, 0.0
#             elif vr == -vl: # special case, rotating in place. r = w/2, v = vr
# #                 print(vr, vl)
#                 return 0.0, vr / (self.WHEEL_SEPARATION / 2) * 0.5     # Seems to rotate slower than predicted wheel velocities
#
#             vo, vi = (vr, vl) if vr > vl else (vl, vr)
#             r = self._get_turn_radius(vo, vi)
#             v = self._get_velocity(vi, r)
#             dir = 1.0 if vr > vl else -1.0
#             return v, dir * v / r
#
#         def arc_with_r(self, r):
#             left_turn = r > 0
#             radius = abs(r)
#
#             w = self.WHEEL_SEPARATION
#             d_outer = (radius + w / 2) * 2 * math.pi
#             d_inner = (radius - w / 2) * 2 * math.pi
#             dt = (d_inner / self.MIN_SPEED)  # time to travers the arc
#
#             v_r, v_l = (d_outer / dt, d_inner / dt) if left_turn else (d_inner / dt, d_outer / dt)
#             return v_r, v_l