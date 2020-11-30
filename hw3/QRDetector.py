"""
    created by Jordan Gassaway, 11/29/2020
    QRDetector: 
"""
from LandmarkDetector import LandmarkDetector

from pyzbar.pyzbar import decode
from pyzbar.wrapper import ZBarSymbol


class QRDetector(LandmarkDetector):
    LANDMARKS = {b'code 0': LandmarkDetector.Landmark((0.65,1.0), 16.0, 'landmark 0 (code0)', 'bottle', b'code 0'),
                 b'code 1': LandmarkDetector.Landmark((1.46,1.0), 16.0, 'landmark 1 (code1)', 'apple', b'code 1'),
                 b'code 2': LandmarkDetector.Landmark((0.26,-0.66), 16.0, 'landmark 2 (code2)', 'tie', b'code 2'),
                 b'code 3': LandmarkDetector.Landmark((1.45,0), 16.0, 'landmark 3 (code3)', 'bowl', b'code 3'),
                 b'code 4': LandmarkDetector.Landmark((1.0,0.39), 16.0, 'landmark 4 (code4)', 'stop sign', b'code 4'),
                 b'code 5': LandmarkDetector.Landmark((0,0.54), 16.0, 'landmark 5 (code5)', 'clock', b'code 5')}

    def __init__(self):
        pass

    def calibrate(self, get_image_func):
        print('Beginning QR Detector Calibration')
        import time
        lmk = self.LANDMARKS['code 0']
        focal_lengths = []

        for d in [0.25, 0.5, 1.0, 1.25]:
            input("Place {} {:.2f}m from robot then press any key to continue".format(lmk.name, d))

            timeout = 5
            time_start = time.time()
            while time.time() - time_start < timeout:
                landmarks = self.detect_landmarks(get_image_func())
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

    def detect_landmarks(self, image):
        def get_bbox(code):
            x1, y1 = code.rect.left, code.rect.top
            x2, y2 = x1 + code.rect.width, y1 + code.rect.height
            return (x1, y1, x2, y2)

        codes = decode(image, symbols=[ZBarSymbol.QRCODE])
#         print("codes: ", codes)

        detections = [LandmarkDetector.Detection(get_bbox(code), code.data) for code in codes]
        landmarks = {}
        for det in detections:
            lmk = self._get_landmark(det)
            if lmk is not None:
                landmarks[lmk.label] = ((lmk, det))

        return landmarks
