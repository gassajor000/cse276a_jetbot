"""
    created by Jordan Gassaway, 10/29/2020
    TestPositionLocator: Test TestPositionLocator Class
"""

import math
from unittest import TestCase
import mock

MODULES = {'filterpy': mock.MagicMock(name='mock filterpy'), 'filterpy.kalman': mock.MagicMock(name='mock kalman')}

with mock.patch.dict('sys.modules', MODULES):
    from hw2.PositionDetector import PositionDetector


class TestPositionLocator(TestCase):
    def setUp(self):
        self.locator = PositionDetector.PositionLocater(PositionDetector.MEASUREMENT_NOISE, PositionDetector.ESTIMATION_PROXIMITY)

    def test_get_position_from_landmarks_no_circles(self):
        """
        get_position_from_landmarks returns the estimated position when there are no landmarks
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position
            ([], (5, 6)),
            ([], (14.6, 8)),
            ([], (3, 7))
        ]

        for landmarks, est_position in test_cases:
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            self.assertEqual(est_position, refined_pos, "Locator did not return the estimated position!")
            self.assertEqual(PositionDetector.PositionLocater.CONFIDENCE_0LM, confidence,
                             "Locator did not indicate a 0 landmark confidence!")

    def test_get_position_from_landmarks_one_circle(self):
        """
        get_position_from_landmarks returns the point closest to the estimated position that is dist away from the landmark.
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 4)], (5, 0), (4, 0)),    # outside circle
            ([((0, 0), 4)], (3, 0), (4, 0)),    # inside circle
            ([((0, 0), 4)], (4, 0), (4, 0)),    # on circle
            ([((3, 5), 2)], (3, 6), (3.0, 7.0)),    # offset from origin
            ([((0, 0), 4)], (5, 5), (4 /math.sqrt(2), 4/math.sqrt(2))),    # 45 degrees
        ]

        for landmarks, est_position, correct_pos in test_cases:
            print("Testing: ", landmarks, est_position, correct_pos)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            self.assertAlmostEqual(correct_pos[0], refined_pos[0], places=4, msg="Locator did not return the correct position!")
            self.assertAlmostEqual(correct_pos[1], refined_pos[1], places=4, msg="Locator did not return the correct position!")
            self.assertEqual(confidence, PositionDetector.PositionLocater.CONFIDENCE_1LM,
                             "Locator did not indicate a 1 landmark confidence!")

    def test_get_position_from_landmarks_two_circles_two_intersects(self):
        """
        get_position_from_landmarks returns a point close to the intersect of the two circles and the estimated position
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 0.5), ((1, 0), 1)], (0.25, 0.4), (0.125, 0.4841)),    # simple case
            ([((1.6, 0.6), 0.5), ((2.5, 1.4), 1)], (2, 0.6), (2.087, 0.487)),    # simple case
            ([((1.6, 0.6), 0.5), ((2.5, 1.4), 1)], (1.5, 1.2), (1.546, 1.097)),    # simple case
        ]

        for landmarks, est_position, correct_pos in test_cases:
            print("Testing: ", landmarks, est_position, correct_pos)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            print(refined_pos)
            self.assertIsNotNone(refined_pos, "Locator was not able to find a position!")
            self.assertAlmostEqual(correct_pos[0], refined_pos[0], delta=0.1, msg="Locator did not return the correct position!")
            self.assertAlmostEqual(correct_pos[1], refined_pos[1], delta=0.1, msg="Locator did not return the correct position!")
            self.assertEqual(PositionDetector.PositionLocater.CONFIDENCE_2LM, confidence,
                             "Locator did not indicate a 1 landmark confidence!")

    def test_get_position_from_landmarks_two_circles_one_intersect(self):
        """
        get_position_from_landmarks returns a point close to the intersect of the two circles and the estimated position
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 0.5), ((1, 0), 0.5)], (0.4, 0.2), (0.5, 0)),    # simple case
            ([((1, 1), 0.25), ((1, 1.5), 0.25)], (1, 1), (1, 1.25)),    # simple case
            ([((1, 1), 0.25), ((1, 1.5), 0.248)], (1, 1), (1, 1.25)),    # circles don't quite intersect
        ]

        for landmarks, est_position, correct_pos in test_cases:
            print("Testing: ", landmarks, est_position, correct_pos)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            print(refined_pos)
            self.assertIsNotNone(refined_pos, "Locator was not able to find a position!")
            self.assertAlmostEqual(correct_pos[0], refined_pos[0], delta=0.1, msg="Locator did not return the correct position!")
            self.assertAlmostEqual(correct_pos[1], refined_pos[1], delta=0.1, msg="Locator did not return the correct position!")
            self.assertEqual(PositionDetector.PositionLocater.CONFIDENCE_2LM, confidence,
                             "Locator did not indicate a 1 landmark confidence!")

    def test_get_position_from_landmarks_two_circles_zero_intersects(self):
        """
        get_position_from_landmarks errors out if the circles don't intersect
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 0.5), ((2, 2), 0.5)], (0.4, 0.4)),    # simple case
            ([((0, 0), 0.5), ((1, 0), 0.5)], (0.8, 0.8)),   # estimated position too far away
        ]

        for landmarks, est_position in test_cases:
            print("Testing: ", landmarks, est_position)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            print(refined_pos)
            self.assertIsNone(refined_pos, "Locator returned a position with bad data!")
            self.assertIsNone(confidence, "Locator returned a confidence with bad data!")

    def test_get_position_from_landmarks_three_circles_one_intersect(self):
        """
        get_position_from_landmarks returns a point close to the intersect of the three circles
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 0.5), ((0.5, 1), 1), ((1, -1), math.sqrt(5)/2)], (0.4, 0), (0.5, 0)),    # simple case
            ([((0, 0), 0.5), ((0.5, 1), 1), ((1, -1), math.sqrt(5)/2)], (5, 5), (0.5, 0)),    # ignores estimated position
            ([((0, 0), 0.5), ((0.5, 1), 0.95), ((1, -1), math.sqrt(5)/2)], (0.4, 0), (0.5, 0)),    # not quite intersect
        ]

        for landmarks, est_position, correct_pos in test_cases:
            print("Testing: ", landmarks, est_position, correct_pos)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            print(refined_pos, confidence)
            self.assertIsNotNone(refined_pos, "Locator was not able to find a position!")
            self.assertAlmostEqual(correct_pos[0], refined_pos[0], delta=0.1, msg="Locator did not return the correct position!")
            self.assertAlmostEqual(correct_pos[1], refined_pos[1], delta=0.1, msg="Locator did not return the correct position!")
            self.assertEqual(PositionDetector.PositionLocater.CONFIDENCE_3LM, confidence,
                             "Locator did not indicate a 1 landmark confidence!")

    def test_get_position_from_landmarks_three_circles_six_intersects(self):
        """
        get_position_from_landmarks returns a point close to the intersect of the three circles and closest to the estimated position
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 0.5), ((0.5, 1), 1), ((1, -1), math.sqrt(5))], (0.4, 0), (0.5, 0)),    # simple case
            ([((0, 0), 0.5), ((0.5, 1), 1), ((1, -1), math.sqrt(5))], (-.4, 0.8), (-.455, 0.703)),    # move estimated position
        ]

        for landmarks, est_position, correct_pos in test_cases:
            print("Testing: ", landmarks, est_position, correct_pos)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            print(refined_pos, confidence)
            self.assertIsNotNone(refined_pos, "Locator was not able to find a position!")
            self.assertAlmostEqual(correct_pos[0], refined_pos[0], delta=0.1, msg="Locator did not return the correct position!")
            self.assertAlmostEqual(correct_pos[1], refined_pos[1], delta=0.1, msg="Locator did not return the correct position!")
            self.assertEqual(PositionDetector.PositionLocater.CONFIDENCE_2LM, confidence,
                             "Locator did not indicate a 1 landmark confidence!")

    def test_get_position_from_landmarks_three_circles_zero_intersects(self):
        """
        get_position_from_landmarks errors out if none of the circles overlap or no 2 circle intersects are
        close to the estimated position
        """
        test_cases = [  # list of landmarks/measurement pairs, estimated position, correct_position
            ([((0, 0), 0.25), ((1, 1), .25), ((2, 2), .25)], (0.4, 0)),    # No intersections
            ([((0, 0), 0.5), ((0.5, 1), 1), ((1, -1), math.sqrt(5))], (5, 5)),    # far from estimated position
        ]

        for landmarks, est_position in test_cases:
            print("Testing: ", landmarks, est_position)
            refined_pos, confidence = self.locator.get_position_from_landmarks(landmarks, est_position)
            print(refined_pos, confidence)
            self.assertIsNone(refined_pos, "Locator returned a position with bad data!")
            self.assertIsNone(confidence, "Locator returned a confidence with bad data!")