"""
    created by Jordan Gassaway, 10/29/2020
    TestPositionDetector: Test PositionDetector Class
"""

import math
from unittest import TestCase
import mock

MOCK_DETECTOR = mock.MagicMock(name='mock QRDetector', get_num_landmarks=mock.MagicMock(return_value=1))

MODULES = {'filterpy': mock.MagicMock(name='mock filterpy'), 'filterpy.kalman': mock.MagicMock(name='mock kalman'),
           'Camera': mock.MagicMock(name='mock Camera'),
           'QRDetector': mock.MagicMock(QRDetector=mock.MagicMock(return_value=MOCK_DETECTOR)),
           'LMObjectDetector': mock.MagicMock(LMObjectDetector=mock.MagicMock(return_value=MOCK_DETECTOR)),
           }

with mock.patch.dict('sys.modules', MODULES):
    from hw3.PositionDetector import PositionDetector


class TestPositionDetector(TestCase):
    RAD_90 = math.radians(90)
    RAD_45 = math.radians(45)
    RAD_30 = math.radians(30)
    SQRT_2 = math.sqrt(2)
    SQRT_3 = math.sqrt(3)

    def setUp(self):
        self.detector = PositionDetector()

    def test_get_z_vector(self):
        """
        test_get_z_vector correctly returns the coordinates relative to the robot
        """
        test_cases = [  # position, measurement list, expected relative coordinates, case msg
            ((0, 0, 0), [(1, -self.RAD_45)], (self.SQRT_2/2, self.SQRT_2/2), 'simple case'),
            ((0, 0, self.RAD_45), [(1, -self.RAD_45)], (0, 1), 'robot angled'),
            ((0, 0, self.RAD_45), [(2, -self.RAD_45)], (0, 2), 'd != 1'),
            ((2, 3, 0), [(1, -self.RAD_45)], (self.SQRT_2/2, self.SQRT_2/2), 'robot offset'),
            ((2, 3, self.RAD_30), [(1, math.radians(-45))], (math.sin(math.radians(15)), math.cos(math.radians(15))), 'robot angled and offset'),
            ((0, 0, -self.RAD_30), [(1, 2*self.RAD_30)], (-0.5, self.SQRT_3/2), 'angled other way'),
        ]

        for pos, measurements, expected, msg in test_cases:
            print("Test: {}".format(msg))
            self.detector.filter.x = pos
            actual = self.detector.get_z_vector(measurements)
            self.assertAlmostEqual(expected[0], actual[0], delta=0.001, msg="Locator did not return the correct x coordinate!")
            self.assertAlmostEqual(expected[1], actual[1], delta=0.001, msg="Locator did not return the correct y coordinate!")
