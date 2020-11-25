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

def take_images():
    def save_image(image):
        import uuid
        file_path = 'images/' + str(uuid.uuid1()) + '.jpg'
        with open(file_path, 'wb') as f:
            f.write(image)

    from jetbot import Camera, bgr8_to_jpeg
    camera = Camera.instance(width=300, height=300)

    import subprocess
    subprocess.call(['mkdir', '-p', 'images'])
    for _ in range(5):
        input('Move robot to a new position. Press any key to continue.')
        save_image(bgr8_to_jpeg(camera.value))

    camera.stop()


if __name__ == '__main__':
    # take_images()

    from PositionDetector import PositionDetector

    p = PositionDetector()
    try:
        p.calibrate()
    finally:
        p.close()

    # W = WallE()
    # W.calibrate()
    # W.drive_to(-1, 0, 0)
    # time.sleep(1)
    # W.drive_to(-1, 1, 1.57)
    # time.sleep(1)
    # W.drive_to(-2, 1, 0)

    # drive_to_waypoints(W)

