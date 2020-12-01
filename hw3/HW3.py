"""
    created by Jordan Gassaway, 10/27/2020
    HW1: Test script to drive WallE to the waypoints
"""
import time

from WallE import WallE

def drive_to_waypoints(file_name):

    # open way points file
    waypoints = []
    with open('circle_path.txt') as fl:
        for line in fl.readlines():
            if line != '' and line != ' ':
                x, y, = line.split(',')
                waypoints.append((float(x), float(y)))

    walle = WallE(init_pos=waypoints[0])
    try:
        input("Please move WallE to ({:.2f}, {:.2f}, 0.00) and press enter".format(*waypoints[0]))
        walle.drive_path(waypoints)
        walle.locator.dump_x()
    finally:
        walle.close()

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

    drive_to_waypoints('circle_path.txt')
    drive_to_waypoints('figure_eight_path.txt')

