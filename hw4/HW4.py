"""
    created by Jordan Gassaway, 10/27/2020
    HW1: Test script to drive WallE to the waypoints
"""
import time

from WallE import WallE

def drive_to_stops(file_name):

    # open way points file
    stops = []
    with open(file_name) as fl:
        for line in fl.readlines():
            if line != '' and line != ' ':
                x, y = line.split(',')
                stops.append((float(x), float(y)))

    start = stops.pop()
    pos = (start[0], start[1], 0.00)
    walle = WallE(init_pos=pos)
    try:
        input("Please move WallE to ({:.2f}, {:.2f}, 0.00) and press enter".format(*stops[0]))
        for stop in stops:
            walle.navigate_to(stop)
            walle.locator.dump_x()
            time.sleep(2.0)
    finally:
        walle.close()


if __name__ == '__main__':
    drive_to_stops('stops.txt')

