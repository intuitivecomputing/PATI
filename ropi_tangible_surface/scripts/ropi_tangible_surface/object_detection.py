#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2

class DetectedObject(TrackerBase):
    def __init__(self):
        super(DetectedObject, self).__init__()
        