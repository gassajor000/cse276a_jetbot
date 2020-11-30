"""
    created by Jordan Gassaway, 11/25/2020
    LandmarkDetector: Detects landmarks in an image
"""
import math
import numpy

from hw3.LandmarkDetector import LandmarkDetector
from jetbot import ObjectDetector


class LMObjectDetector(LandmarkDetector):
    # Landmark positions & classes
    LANDMARKS = {44: LandmarkDetector.Landmark((0.65,1.0), 22.5, 'landmark 0 [olive oil]', 'bottle', 44),
                 53: LandmarkDetector.Landmark((1.46,1.0), 7.5, 'landmark 1 [apple]', 'apple', 53),
                 32: LandmarkDetector.Landmark((0.26,-0.66), 60.0, 'landmark 2 [tie]', 'tie', 32),
                 51: LandmarkDetector.Landmark((1.45,0), 11.5, 'landmark 3 [bowl]', 'bowl', 51),
                 13: LandmarkDetector.Landmark((1.0,0.39), 23.0, 'landmark 4 [stop sign]', 'stop sign', 32),
                 85: LandmarkDetector.Landmark((0,0.54), 13.6, 'landmark 5 [clock]', 'clock', 85)}

    def __init__(self, model=None, model_path='/home/jetbot/Notebooks/object_following/'):
        super().__init__(model, model_path)
        if model:
            self.model = model
        else:
            print("Initializing Model...")
            self.model = ObjectDetector(model_path + 'ssd_mobilenet_v2_coco.engine')

    def calibrate(self, get_image_func):
        print('Beginning Object Detector Calibration')
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
            f = self.get_focal_length(d, height_on_sensor_cm, lmk)
            print("height on sensor {}, f {}".format(height_on_sensor_cm, f))
            focal_lengths.append(f)


        f = float(numpy.mean(focal_lengths))
        print('Average focal length = {:.3f}'.format(f))
        self.FOCAL_LENGTH = f

    def detect_landmarks(self, image):
        objs = self.model(image)[0]
        detections = [LandmarkDetector.Detection(obj['bbox'], obj['label']) for obj in objs]
        landmarks = {}
        for det in detections:
            lmk = self._get_landmark(det)
            if lmk is not None:
                landmarks[lmk.label] = ((lmk, det))

        return landmarks
