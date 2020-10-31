"""
    created by Jordan Gassaway, 10/27/2020
    HW1: Test script to drive WallE to the waypoints
"""
import time

from WallE import WallE

if __name__ == '__main__':
    W = WallE()
    W.calibrate()
    input('press any key to start sequence')
    W.drive_to(-1, 0, 0)
    time.sleep(1)
    W.drive_to(-1, 1, 1.57)
    time.sleep(1)
    W.drive_to(-2, 1, 0)

    # # open way points file
    # waypoints = []
    #
    # for point in waypoints:
    #     x, y, theta = point
    #     WALLE.drive_to(x, y, theta)
    #     time.sleep(1)