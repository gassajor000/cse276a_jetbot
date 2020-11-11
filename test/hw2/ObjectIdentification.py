"""
    created by Jordan Gassaway, 11/10/2020
    ObjectFollowDemo: Object follow with collision avoidance and motor control stripped out.
        (Just object detection and identification)
"""

# Setup object detector
from jetbot import ObjectDetector

model = ObjectDetector('ssd_mobilenet_v2_coco.engine')

# Setup camera
from jetbot import Camera

camera = Camera.instance(width=300, height=300)
detections = model(camera.value)

print(detections)

# Code to display objects in jupyter notebook
from IPython.display import display
import ipywidgets.widgets as widgets

detections_widget = widgets.Textarea()

detections_widget.value = str(detections)

display(detections_widget)

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
label_widget = widgets.IntText(value=1, description='tracked label')
speed_widget = widgets.FloatSlider(value=0.4, min=0.0, max=1.0, description='speed')
turn_gain_widget = widgets.FloatSlider(value=0.8, min=0.0, max=2.0, description='turn gain')

display(widgets.VBox([
    label_widget,
    speed_widget,
    turn_gain_widget
]))

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


def closest_detection(detections):
    """Finds the detection closest to the image center"""
    closest_detection = None
    for det in detections:
        center = detection_center(det)
        if closest_detection is None:
            closest_detection = det
        elif norm(detection_center(det)) < norm(detection_center(closest_detection)):
            closest_detection = det
    return closest_detection


def execute(change):
    image = change['new']

    # compute all detected objects
    detections = model(image)

    # draw all detections on image
    for det in detections[0]:
        bbox = det['bbox']
        cv2.rectangle(image, (int(width * bbox[0]), int(height * bbox[1])),
                      (int(width * bbox[2]), int(height * bbox[3])), (255, 0, 0), 2)

    # select detections that match selected class label
    matching_detections = [d for d in detections[0] if d['label'] == int(label_widget.value)]

    # get detection closest to center of field of view and draw it
    det = closest_detection(matching_detections)
    if det is not None:
        bbox = det['bbox']
        cv2.rectangle(image, (int(width * bbox[0]), int(height * bbox[1])),
                      (int(width * bbox[2]), int(height * bbox[3])), (0, 255, 0), 5)

    # update image widget
    image_widget.value = bgr8_to_jpeg(image)


execute({'new': camera.value})


# call execute on camera update
camera.unobserve_all()
camera.observe(execute, names='value')


# manual disconnect
import time

camera.unobserve_all()
time.sleep(1.0)