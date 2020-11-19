"""
    created by Jordan Gassaway, 11/10/2020
    ObjectFollowDemo: Object follow with collision avoidance and motor control stripped out.
        (Just object detection and identification)
"""

# Setup object detector
from jetbot import ObjectDetector

model = ObjectDetector('/home/jetbot/Notebooks/object_following/ssd_mobilenet_v2_coco.engine')

# Setup camera
from jetbot import Camera

camera = Camera.instance(width=300, height=300)
detections = model(camera.value)


# Code to display objects in jupyter notebook
from IPython.display import display
import ipywidgets.widgets as widgets


image_number = 0
object_number = 0


# Script to control robot to follow obj
import cv2
import numpy as np


mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])


# display widgets and setup control
from jetbot import bgr8_to_jpeg

image_widget = widgets.Image(format='jpeg', width=300, height=300)
# object_widget = widgets.IntText(value=1, description='tracked label')

display(widgets.VBox([
    image_widget,
    # label_widget,
]))

# Import PositionDetector
from PositionDetector import PositionDetector

p = PositionDetector(model=model, camera=camera)

width = int(image_widget.width)
height = int(image_widget.height)


def detection_center(detection):
    """Computes the center x, y coordinates of the object"""
    bbox = detection['bbox']
    center_x = (bbox[0] + bbox[2]) / 2.0 - 0.5
    center_y = (bbox[1] + bbox[3]) / 2.0 - 0.5
    return (center_x, center_y)


def norm(vec):
    """Computes the length of the 2D vector"""
    return np.sqrt(vec[0] ** 2 + vec[1] ** 2)


counter = 0
frequency = 4  # every other frame


def execute(change):
    global counter, frequency
    if counter % frequency == 0:
        image = change['new']
        image_fixed = p.undistort_image(image)  # undistort the image

        # get landmarks
        detections = model(image_fixed)[0]
        landmarks_detected = p.detector.detect_landmarks(detections)

        if landmarks_detected:
            for lmk, det in landmarks_detected.values():
                dist = p.detector.get_distance_to_landmark(lmk, det)
                ang = p.detector.get_angle_offset_to_landmark(det)

                bbox = det['bbox']
                cv2.rectangle(image_fixed, (int(width * bbox[0]), int(height * bbox[1])),
                              (int(width * bbox[2]), int(height * bbox[3])), (255, 0, 0), 2)

                cv2.putText(image_fixed, "{}".format(lmk.name),
                            (int(width * bbox[0]), int(height * bbox[1]) + 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,  # font scale
                            (255, 255, 255),  # font color
                            1)  # line type
                cv2.putText(image_fixed, "{:.2f}cm".format(dist * 100),
                            (int(width * bbox[0]), int(height * bbox[1]) + 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,  # font scale
                            (255, 255, 255),  # font color
                            1)  # line type

                cv2.putText(image_fixed, "{:.2f}deg".format(np.rad2deg(ang)),
                            (int(width * bbox[0]), int(height * bbox[1]) + 65),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,  # font scale
                            (255, 255, 255),  # font color
                            1)  # line type

        # update image widget
        image_widget.value = bgr8_to_jpeg(image_fixed)

    counter += 1

# call execute on camera update
camera.unobserve_all()
camera.observe(execute, names='value')


# manual disconnect
import time

camera.unobserve_all()
time.sleep(1.0)