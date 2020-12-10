"""
    created by Jordan Gassaway, 11/25/2020
    LandmarkDetector: Detects landmarks in an image
"""
import math

class Landmark():
    def __init__(self, position, height, name, category, label):
        self.label = label
        self.name = name
        self.category = category
        self.height = height  # cm
        self.position = position


class LandmarkDetector():
    def __init__(self, landmarks):
        self.LANDMARKS = landmarks

    class Detection():
        def __init__(self, bounding_box, label):
            self.x1, self.y1, self.x2, self.y2 = bounding_box
            self.label = label

    # Landmark positions & classes
    LANDMARKS = {}  # type: Dict[bytes, Landmark]

    CAMERA_OFFSET = 2.0  # cm between the camera and the position model point
    FOCAL_LENGTH = .178     # 1.78 mm
    PIXEL_SIZE = 0.0009199   # (cm) 9.199 um /pixel
    RAD_PER_PIXEL = math.radians(136) / 300   # degrees offset from straight on per pixel offset

    @staticmethod
    def get_focal_length(d, height_on_sensor_cm, lmk: Landmark):
        return (d * 100) * height_on_sensor_cm / lmk.height

    def calibrate(self, get_image_func):
        """
        For each landmark, measure bounding box at 0.25, 0.5, 1.0, and 1.25
        print results to screen for computation
        """
        raise  NotImplementedError('Implemented by subclass!')

    def get_height_on_sensor(self, detection:Detection):
        """Return the height of the detection on the sensor in cm"""
        height_pixels = abs(detection.y1 - detection.y2)  # Scale to height of image
#             print("{} pixels high. bbox {}".format(height_pixels, detection['bbox']))
        return self.PIXEL_SIZE * height_pixels

    def get_offset_from_center(self, detection: Detection):
        """Return the number of pixels detection is offset from center. Positive = right/ negative = left"""
        obj_center = ((detection.x1 + detection.x2) / 2.0)
#             print(obj_center)
        return 150 - obj_center

    def detect_landmarks(self, image):
        """
        Return a list of all landmarks currently visible, paired with the detection data, keyed by the landmark label
        """
        raise NotImplementedError("Implemented by subclass!")

    def _get_landmark(self, detection: Detection):
        """
        returns the landmark corresponding to the object detected.
        Returns None if object does not match any landmarks

        :param detection: object detection returned from jetbot Object Detector
        """
        return self.LANDMARKS.get(detection.label, None)

    def get_distance_to_landmark(self, landmark: Landmark, detection: Detection):
        """
        :param landmark: one of the landmarks in self.LANDMARKS
        :param detection: object detection returned from jetbot Object Detector
        :return: the estimated distance to landmark (in meters)
        """

        height_on_sensor_cm = self.get_height_on_sensor(detection)

        # Use similar triangles to estimate distance
        d_cm = landmark.height * self.FOCAL_LENGTH / height_on_sensor_cm

        return d_cm / 100   # convert to meters

    def get_angle_offset_to_landmark(self, detection: Detection):
        """
        Get the angle offset from the direction the robot is looking to the object
        :param detection: object detection returned from jetbot Object Detector
        :return: offset (radians). Positive = left, Negative = right
        """
        return self.get_offset_from_center(detection) * self.RAD_PER_PIXEL

    def get_num_landmarks(self):
        """Returns the number of landmarks registered to the detector"""
        return len(self.LANDMARKS)

    def get_landmark_labels(self):
        """Returns a list of all the landmark labels"""
        return list(self.LANDMARKS.keys())

    def get_landmark_positions(self):
        """Returns a mapping of each of the landmark positions"""
        return {id: l.position for id, l in self.LANDMARKS.items()}