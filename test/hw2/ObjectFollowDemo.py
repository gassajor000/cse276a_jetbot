"""
    created by Jordan Gassaway, 11/10/2020
    ObjectFollowDemo: copied from jetbot jupyter notebook

    # Object Following - Live Demo

    In this notebook we'll show how you can follow an object with JetBot!  We'll use a pre-trained neural network
    that was trained on the [COCO dataset](http://cocodataset.org) to detect 90 different common objects.  These include

    * Person (index 0)
    * Cup (index 47)

    and many others (you can check [this file](https://github.com/tensorflow/models/blob/master/research/object_detection/data/mscoco_complete_label_map.pbtxt) for a full list of class indices).  The model is sourced from the [TensorFlow object detection API](https://github.com/tensorflow/models/tree/master/research/object_detection),
    which provides utilities for training object detectors for custom tasks also!  Once the model is trained, we optimize it using NVIDIA TensorRT on the Jetson Nano.

    This makes the network very fast, capable of real-time execution on Jetson Nano!  We won't run through all of the training and optimization steps in this notebook though.

    Anyways, let's get started.  First, we'll want to import the ``ObjectDetector`` class which takes our pre-trained SSD engine.
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
import torch
import torchvision
import torch.nn.functional as F
import cv2
import numpy as np

collision_model = torchvision.models.alexnet(pretrained=False)
collision_model.classifier[6] = torch.nn.Linear(collision_model.classifier[6].in_features, 2)
collision_model.load_state_dict(torch.load('../collision_avoidance/best_model.pth'))
device = torch.device('cuda')
collision_model = collision_model.to(device)

mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])

normalize = torchvision.transforms.Normalize(mean, stdev)

def preprocess(camera_value):
    global device, normalize
    x = camera_value
    x = cv2.resize(x, (224, 224))
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]
    return x


# init motors
from jetbot import Robot

robot = Robot()


# display widgets and setup control
from jetbot import bgr8_to_jpeg

blocked_widget = widgets.FloatSlider(min=0.0, max=1.0, value=0.0, description='blocked')
image_widget = widgets.Image(format='jpeg', width=300, height=300)
label_widget = widgets.IntText(value=1, description='tracked label')
speed_widget = widgets.FloatSlider(value=0.4, min=0.0, max=1.0, description='speed')
turn_gain_widget = widgets.FloatSlider(value=0.8, min=0.0, max=2.0, description='turn gain')

display(widgets.VBox([
    widgets.HBox([image_widget, blocked_widget]),
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

    # execute collision model to determine if blocked
    collision_output = collision_model(preprocess(image)).detach().cpu()
    prob_blocked = float(F.softmax(collision_output.flatten(), dim=0)[0])
    blocked_widget.value = prob_blocked

    # turn left if blocked
    if prob_blocked > 0.5:
        robot.left(0.3)
        image_widget.value = bgr8_to_jpeg(image)
        return

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

    # otherwise go forward if no target detected
    if det is None:
        robot.forward(float(speed_widget.value))

    # otherwsie steer towards target
    else:
        # move robot forward and steer proportional target's x-distance from center
        center = detection_center(det)
        robot.set_motors(
            float(speed_widget.value + turn_gain_widget.value * center[0]),
            float(speed_widget.value - turn_gain_widget.value * center[0])
        )

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
robot.stop()