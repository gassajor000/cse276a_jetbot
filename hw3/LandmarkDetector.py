"""
    created by Jordan Gassaway, 11/25/2020
    LandmarkDetector: Detects landmarks in an image
"""
import math
import numpy

from jetbot import ObjectDetector


class LandmarkDetector():
    class Landmark():
        def __init__(self, position, height, name, category, label):
            self.label = label
            self.name = name
            self.category = category
            self.height = height    # cm
            self.position = position

    # Landmark positions & classes
    LANDMARKS = {44: Landmark((0,0), 22.5, 'landmark 0 [olive oil]', 'bottle', 44),
                 53: Landmark((0,0), 7.5, 'landmark 1 [apple]', 'apple', 53),
                 32: Landmark((0,0), 150, 'landmark 2 [tie]', 'tie', 32),
                 51: Landmark((0,0), 11.5, 'landmark 3 [bowl]', 'bowl', 51),
                 13: Landmark((0,0), 23.0, 'landmark 4 [stop sign]', 'stop sign', 32),
                 85: Landmark((0,0), 13.6, 'landmark 5 [clock]', 'clock', 85)}

    CAMERA_OFFSET = 2.0  # cm between the camera and the position model point
    FOCAL_LENGTH = .159     # 1.59 mm
    PIXEL_SIZE = 0.0009199   # (cm) 9.199 um /pixel
    RAD_PER_PIXEL = math.radians(136) / 300   # degrees offset from straight on per pixel offset

    def __init__(self, model=None, model_path='/home/jetbot/Notebooks/object_following/'):
        if model:
            self.model = model
        else:
            print("Initializing Model...")
            self.model = ObjectDetector(model_path + 'ssd_mobilenet_v2_coco.engine')

    def calibrate(self, get_image_func):
        """
        For each landmark, measure bounding box at 0.25, 0.5, 1.0, and 1.25
        print results to screen for computation
        """
        def get_focal_length():
            return (d * 100) * height_on_sensor_cm / lmk.height

        print('Beginning LandmarkDetector Calibration')
        import time
        lmk = self.LANDMARKS[44]
        focal_lengths = []

        for d in [0.25, 0.5, 1.0, 1.25]:
            input("Place {} {:.2f}m from robot then press any key to continue".format(lmk.name, d))

            timeout = 5
            time_start = time.time()
            while time.time() - time_start < timeout:
                detections = self.model(get_image_func())
                landmarks = self.detect_landmarks(detections[0])
                lmk1, det = landmarks.get(lmk.label, (None, None))
                if lmk1:
                    break
                else:
                    time.sleep(.02)
            else:
                raise RuntimeError('{} was not detected'.format(lmk.name))

            print(lmk1)
            height_on_sensor_cm = self.get_height_on_sensor(det)
            f = get_focal_length()
            print("height on sensor {}, f {}".format(height_on_sensor_cm, f))
            focal_lengths.append(f)


        f = float(numpy.mean(focal_lengths))
        print('Average focal length = {:.3f}'.format(f))
        self.FOCAL_LENGTH = f

    def get_height_on_sensor(self, detection):
        """Return the height of the detection on the sensor in cm"""
        x1, y1, x2, y2 = detection['bbox']
        height_pixels = abs(y1 - y2) * 300    # Scale to height of image
#             print("{} pixels high. bbox {}".format(height_pixels, detection['bbox']))
        return self.PIXEL_SIZE * height_pixels

    def get_offset_from_center(self, detection):
        """Return the number of pixels detection is offset from center. Positive = right/ negative = left"""
        x1, y1, x2, y2 = detection['bbox']
        obj_center = ((x1 + x2) / 2.0) * 300
#             print(obj_center)
        return obj_center - 150

    def detect_landmarks(self, image):
        """
        Return a list of all landmarks currently visible, paired with the detection data, keyed by the landmark label
        """
        detections = self.model(image)
        landmarks = {}
        for det in detections:
            lmk = self._get_landmark(det)
            if lmk is not None:
                landmarks[lmk.label] = ((lmk, det))

        return landmarks

    def _get_landmark(self, detection):
        """
        returns the landmark corresponding to the object detected.
        Returns None if object does not match any landmarks

        :param detection: object detection returned from jetbot Object Detector
        """
        return self.LANDMARKS.get(detection['label'], None)

    def get_distance_to_landmark(self, landmark: Landmark, detection):
        """
        :param landmark: one of the landmarks in self.LANDMARKS
        :param detection: object detection returned from jetbot Object Detector
        :return: the estimated distance to landmark (in meters)
        """

        height_on_sensor_cm = self.get_height_on_sensor(detection)

        # Use similar triangles to estimate distance
        d_cm = landmark.height * self.FOCAL_LENGTH / height_on_sensor_cm

        return d_cm / 100   # convert to meters

    def get_angle_offset_to_landmark(self, detection):
        """
        Get the angle offset from the direction the robot is looking to the object
        :param detection: object detection returned from jetbot Object Detector
        :return: offset (radians). Positive = right, Negative = left
        """
        return self.get_offset_from_center(detection) * self.RAD_PER_PIXEL