#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import message_filters
import copy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ropi_tangible_surface.transform import four_point_transform
from ropi_tangible_surface.fingertip_detection import *
import cv2
from ropi_msgs.msg import MultiTouch
from geometry_msgs.msg import Point

my_threshold = lambda img, l, u: (img < u) * (img > l) * img
flip_array = lambda a: np.hstack((np.hsplit(a, 2)[1], np.hsplit(a, 2)[0]))
tip_angle = lambda tip_pt, pt1, pt2: np.arctan2(pt2[1] - tip_pt[1], pt2[0] - tip_pt[0]) - np.arctan2(pt1[1] - tip_pt[1], pt1[0] - tip_pt[0])
euclidean_dist = lambda pt1, pt2: np.linalg.norm(np.array(pt1) - np.array(pt2))

class TAngibleSurface:
    def __init__(self):
        