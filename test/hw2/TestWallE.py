"""
    created by Jordan Gassaway, 10/29/2020
    TestWallE: Test WallE Class
"""

import math
from unittest import TestCase
import mock

MODULES = {'jetbot': mock.MagicMock(name='mock jetbot')}

with mock.patch.dict('sys.modules', MODULES):
    from hw2.WallE import WallE



class TestWallEMovementModel(TestCase):
    RAD_90 = math.radians(90)
    RAD_180 = math.radians(180)
    RAD_270 = math.radians(270)

    def test_get_drive_time(self):
        test_cases = [  # calibration, distance, time
            ((1, 1),    5,      5),     # simple
            ((.5, 1),    5,     10),   # double
            ((.5, 1),    10,    20),   # double 2
            ((.25, 2),  .5,     2),   # frac dist
            ((1, 2),  .5,     .5),    # frac time
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            calibration, d, t = test_case
            d1, theta = calibration

            mm = WallE.MovementModel()
            mm.calibrate(d1, theta)
            t1 = mm.get_drive_duration(d)
            self.assertAlmostEqual(t, t1, delta= 0.001, msg="Invalid drive time!")

    def test_get_rotation_time(self):
        test_cases = [  # calibration, distance, time
            ((1, 1),    5,      5),     # simple
            ((1, .5),    5,     10),   # double
            ((1, .5),    10,    20),   # double 2
            ((2, .25),  .5,     2),   # frac dist
            ((1, 1),  .5,     .5),    # frac time
        ]

        for test_case in test_cases:
            print('Testing ' + str(test_case))
            calibration, theta, t = test_case
            d1, theta1 = calibration

            mm = WallE.MovementModel()
            mm.calibrate(d1, theta1)
            t1 = mm.get_rotation_time(theta)
            self.assertAlmostEqual(t, t1, delta= 0.001, msg="Invalid rotation time!")
