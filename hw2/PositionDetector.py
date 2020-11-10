"""
    created by Jordan Gassaway, 11/10/2020
    PositionDetector: Uses the camera to detect the current position
"""


class PositionDetector:
    # Landmark position
    LANDMARK_1 = (0,0,0)
    LANDMARK_2 = (0,0,0)
    LANDMARK_3 = (0,0,0)
    LANDMARK_4 = (0,0,0)
    CAMERA_OFFSET = 2.0     # cm between the camera and the position model point


    def __init__(self):
        # todo setup camera & object detection
        pass

    def get_position(self):
        """
        Get the estimate position of the robot
        :return: tuple (x, y, theta) position & orientation of the robot
        """
