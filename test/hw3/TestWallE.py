"""
    created by Jordan Gassaway, 10/29/2020
    TestWallE: Test WallE Class
"""

import math
from unittest import TestCase
import mock

from hw3.PositionModel import PositionModel

MODULES = {'jetbot': mock.MagicMock(name='mock jetbot'), 'time': mock.MagicMock(name='mock time'),
           'hw3.PositionModel': mock.MagicMock(name='mock position model'),
           'hw3.PositionDetector': mock.MagicMock(name='mock position detector'), }

with mock.patch.dict('sys.modules', MODULES):
    from hw3.WallE import WallE

class TestWallEMovementModel(TestCase):
    RAD_90 = math.radians(90)
    RAD_180 = math.radians(180)
    RAD_270 = math.radians(270)

    def test_arc_to_geometry(self):
        """arc to returns a valid arc to the point and neither wheel speed exceeds the max speed."""
        def get_dist(x1, y1, x2, y2):
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        def get_r_theta(vr, vl):
            vo, vi = (vr, vl) if vr > vl else (vl, vr)
            v_ratio = vo/vi
            w = mm.WHEEL_SEPARATION / 2
            r = (w * (v_ratio +1)/(v_ratio -1))
            theta = 2 * w * (r + w) / (1 - 1 / v_ratio)
            return r / 100.0, theta

        def get_center(r, theta, start, end):
            # place robot at (0, 0), point at (d, 0)
            # compute cx, cy
            phi = self.RAD_90 - theta/2
            cx0, cy0 = r * math.cos(phi), r* math.sin(phi)

            # rotate around robot
            theta_rotate = pos.get_abs_angle_to(end[0], end[1]) + self.RAD_90
            cx1 = cx0* math.cos(theta_rotate) - cy0 * math.sin(theta_rotate)
            cy1 = cx0* math.cos(theta_rotate) + cy0 * math.sin(theta_rotate)

            # translate to global origin
            cx2 =  cx1 + start[0]
            cy2 = cy1 + start[1]
            return cx2, cy2

        test_cases = [  # position, point
            ((0, 0, 0),    (-1, 1)),     # simple
            # ((.5, 1),   5,),   # double
            # ((.5, 1),   10,),   # double 2
            # ((.25, 2),  .5,),   # frac dist
            # ((1, 2),    .5,),    # frac time
        ]

        mm = WallE.MovementModel()
        for test_case in test_cases:
            print('Testing ' + str(test_case))
            start, end = test_case
            pos = PositionModel(x=start[0], y=start[1], theta=start[2])

            vr, vl = mm.arc_to(end[0], end[1], pos)
            print("\tvr, vl {}, {}".format(vr, vl))
            r, theta = get_r_theta(vr, vl)
            cx, cy = get_center(r, theta, start, end)
            print("\tCircle center {}, {}".format(cx, cy))

            # make sure center is  r away from both points
            d1, d2 = get_dist(cx, cy, start[0], start[1]), get_dist(cx, cy, end[0], end[1])
            self.assertAlmostEqual(d1, r, delta= 0.001, msg="Invalid circle coordinates")
            self.assertAlmostEqual(d2, r, delta= 0.001, msg="Invalid circle coordinates")

            # make sure its tangent to orientation
            vec_c = (cx - start[0], cy - start[1])
            vec_r = (math.sin(start[2] + self.RAD_90), math.cos(start[2] + self.RAD_90))
            dot_product = vec_c[0] * vec_r[0] + vec_c[1] * vec_r[1]
            self.assertAlmostEqual(dot_product, 0.0, delta= 0.001, msg="Circle was not tangent to orientation!")
