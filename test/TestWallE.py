"""
    created by Jordan Gassaway, 10/29/2020
    TestWallE: Test WallE Class
"""

import math
from unittest import TestCase
import mock

MODULES = {'jetbot': mock.MagicMock(name='mock jetbot')}

with mock.patch.dict('sys.modules', MODULES):
    from hw1.WallE import WallE


def _rad(deg):
    return math.radians(deg)


class TestWallEPositionModel(TestCase):
    RAD_90 = math.radians(90)
    RAD_180 = math.radians(180)
    RAD_270 = math.radians(270)

    def _verify_position(self, pm: WallE.PositionModel, x, y, theta):
        self.assertAlmostEqual(pm.x, x, delta=0.001, msg="X position off!")
        self.assertAlmostEqual(pm.y, y, delta=0.001, msg="Y position off!")
        self.assertAlmostEqual(pm.theta, theta, delta=0.001, msg="Orientation off!")

    def test_move_forward(self):
        test_cases = [  # start, distance, end
            ((0, 0, 0),         10,     (0, 10, 0)),        # facing north
            ((0, 0, self.RAD_90),    10,     (10, 0, self.RAD_90)),  # facing east
            ((0, 0, self.RAD_180),   10,     (0, -10, self.RAD_180)),    # facing west
            ((0, 0, self.RAD_270),   10,     (-10, 0, self.RAD_270)),    # facing south
            ((0, 0, _rad(45)),      10,     (7.071, 7.071, _rad(45))),     # 45 degrees
            ((0, 0, _rad(225)),      10,     (-7.071, -7.071,  _rad(225))),        # 225 degrees
            ((0, 0, _rad(300)),      10,     (-8.66, 5,  _rad(300))),        # 300 degrees
            ((1.5, 2, self.RAD_90),  10,     (11.5, 2,  self.RAD_90)),        # offset Q1
            ((-1.5, -2, _rad(45)),    10,     (5.571, 5.071,  _rad(45))),        # offset Q3
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            start, d, end = test_case
            x1, y1, t1 = start
            x2, y2, t2 = end

            pm = WallE.PositionModel(x1, y1, t1)
            pm.move_forward(d)
            self._verify_position(pm, x2, y2, t2)

    def test_rotate_clockwise(self):
        test_cases = [  # start, theta, end
            ((0, 0, 0),              self.RAD_90,     (0, 0, self.RAD_90)),        # 90 deg
            ((0, 0, self.RAD_90),    self.RAD_90,     (0, 0, self.RAD_180)),   # 90 deg
            ((0, 0, self.RAD_270),   self.RAD_180,    (0, 0, self.RAD_90)),    # wrap around 360
            ((0, 0, _rad(45)),       self.RAD_90,     (0, 0, _rad(135))),     # 45 degrees
            ((0, 0, _rad(300)),      self.RAD_180,    (0, 0, _rad(120))),     # 300 degrees
            ((1, 3, self.RAD_90),    _rad(45),        (1, 3, _rad(135))),        # offset from 0,0
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            start, theta, end = test_case
            x1, y1, t1 = start
            x2, y2, t2 = end

            pm = WallE.PositionModel(x1, y1, t1)
            pm.rotate_clockwise(theta)
            self._verify_position(pm, x2, y2, t2)


    def test_get_distance_to(self):
        test_cases = [  # start, point, dist
            ((0, 0, 0),     (0, 10),     10),        # up
            ((0, 0, 0),     (-10, 0),     10),   # left
            ((0, 0, 0),     (10, 0),     10),    # right
            ((0, 0, self.RAD_90),     (0, -10),     10),     # down
            ((0, 0, 0),     (4, 4),     5.657),     # 45 deg
            ((0, 0, 0),     (-2.5, -4.330),     5),     # 240 deg
            ((-1, -3, 0),  (-3, -5),    2.828),     # offset start 45 deg
            ((1, 3, 0),    (5.330, 5.5),    5),        # offset start 30 deg
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            start, point, d = test_case
            x1, y1, t1 = start
            x2, y2, = point

            pm = WallE.PositionModel(x1, y1, t1)
            d2 = pm.get_distance_to(x2, y2)
            self.assertAlmostEqual(d2, d, delta=0.001, msg='Invalid distance measurement!')

    def test_get_abs_angle_to(self):
        test_cases = [  # start, point, rad
            ((0, 0, 0),     (1, 0),     self.RAD_90),        # right
            ((0, 0, 0),     (0, -1),    self.RAD_180),   # down
            ((0, 0, 0),     (1, 1),     _rad(45)),    # 45 deg
            ((0, 0, self.RAD_90),     (-10, 0),     self.RAD_270),     # right
            ((0, 0, 0),     (-1, 1.732),     _rad(330)),     # 330 deg
            ((1, 3, 0),     (2, 4),     _rad(45)),     # offset 1
            ((-6, -2, 0),   (-5, -0.268),     _rad(30)),     # offset 2
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            start, point, d = test_case
            x1, y1, t1 = start
            x2, y2, = point

            pm = WallE.PositionModel(x1, y1, t1)
            d2 = pm.get_abs_angle_to(x2, y2)
            self.assertAlmostEqual(d2, d, delta=0.001, msg='Invalid angle measurement!')

    def test_get_rel_angle_to(self):
        test_cases = [  # start, point, rad
            ((0, 0, 0),             self.RAD_90, self.RAD_90),      # 90 deg
            ((0, 0, self.RAD_90),   _rad(120),    _rad(30)),        # angled to start
            ((0, 0, _rad(120)),     self.RAD_90,     _rad(330)),    # around the horn
            ((0, 0, _rad(300)),     _rad(30),     self.RAD_90),     # around the horn 2
            ((1, 3, 0),     self.RAD_270,     self.RAD_270),     # offset 1
            ((-6, -2, _rad(45)),   self.RAD_180,     _rad(135)),     # offset 2
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            start, t2, cw_rotation = test_case
            x1, y1, t1 = start

            pm = WallE.PositionModel(x1, y1, t1)
            t3 = pm.get_rel_angle_to(t2)
            self.assertAlmostEqual(t3, cw_rotation, delta=0.001, msg='Invalid rotation measurement!')