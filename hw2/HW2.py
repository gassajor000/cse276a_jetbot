"""
    created by Jordan Gassaway, 10/27/2020
    HW1: Test script to drive WallE to the waypoints
"""
import time

from WallE import WallE

def drive_to_waypoints(walle):
    # open way points file
    waypoints = []
    with open('waypoints.txt') as fl:
        for line in fl.readlines():
            if line != '' and line != ' ':
                x, y, theta = line.split(',')
                waypoints.append((float(x), float(y), float(theta)))

    for point in waypoints:
        x, y, theta = point
        walle.drive_to(x, y, theta)
        time.sleep(1)

if __name__ == '__main__':


    # W = WallE()
    # W.calibrate()
    from PositionDetector import PositionDetector

    p = PositionDetector()
    input('press any key to start sequence')
    p.detector.calibrate(p.model, p.camera)
    # W.drive_to(-1, 0, 0)
    # time.sleep(1)
    # W.drive_to(-1, 1, 1.57)
    # time.sleep(1)
    # W.drive_to(-2, 1, 0)

    # drive_to_waypoints(W)

