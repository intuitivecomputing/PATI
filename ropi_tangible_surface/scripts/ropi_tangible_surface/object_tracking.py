#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

from ropi_tangible_surface.base_classes import *
from ropi_tangible_surface.selection import *
import rospy
import cv2

class ObjectTracker(TrackerBase):
    TYPE = {TYPE_OBJECT_SELECTION: 0, TYPE_AREA_SELECTION: 1}
    resolution = (450, 800)
    def __init__(self, msg):
        super(ObjectTracker, self).__init__()