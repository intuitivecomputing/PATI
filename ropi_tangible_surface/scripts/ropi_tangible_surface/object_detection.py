#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2

class ObjectManager:
    def __init__(self):
        self.init()

    def init(self):
        self.detections = []
        self.depth_img = None
        self.debug_img = None

    def set_debug_img(self, img):
        if img is not None and (len(img.shape) < 3 or img.shape[2] != 3):
            self.debug_img = cv2.cvtColor(
                img.astype(np.uint8)[:, :, np.newaxis], cv2.COLOR_GRAY2BGR)
        elif img is not None:
            self.debug_img = img

    def update(self, cnts, depth_image, debug_img=None):
        self.init()
        output = []
        for cnt in cnts:
            detection = ObjectDetection()
            detection.update(cnt, depth_image, debug_img)
            self.detections.append(detection)
            output.append(detection.center_of_mass)
        return output

class ObjectDetection:
    def __init__(self, debug=True):
        self.cnt = None
        self.hull = None
        self.moments = None
        self.defects = None
        self.center_of_mass = None
        self.object_pos = []
        self.debug = debug
        self.depth_img = None
        self.debug_img = None
        self.window_size = 5

    @staticmethod
    def euclidean_dist(pt1, pt2):
        return np.linalg.norm(np.array(pt1) - np.array(pt2))

    def distance_to_center(self, pt):
        return self.euclidean_dist(pt, self.center_of_mass)

    def set_debug_img(self, img):
        if img is not None and (len(img.shape) < 3 or img.shape[2] != 3):
            self.debug_img = cv2.cvtColor(
                img.astype(np.uint8)[:, :, np.newaxis], cv2.COLOR_GRAY2BGR)
        elif img is not None:
            self.debug_img = img

    def clear(self):
        self.cnt = None
        self.hull = None
        self.moments = None
        self.defects = None
        self.center_of_mass = None
        self.object_pos = []
        self.depth_img = None
        self.debug_img = None

    def update(self, cnt, depth_img, debug_img=None):
        self.clear()
        if self.debug and debug_img is None:
            self.set_debug_img(depth_img)
        if debug_img is not None:
            self.set_debug_img(debug_img)
        self.depth_img = depth_img
        if cv2.contourArea(cnt) < 100:
            rospy.logdebug_throttle(2, 'Contour area too small.')
            return []
        self.cnt = cnt
        self.hull = cv2.convexHull(cnt, returnPoints=False)
        # calculate moments and center of mass
        self.moments = cv2.moments(cnt)
        if self.moments['m00'] != 0:
            cx = int(self.moments['m10'] / self.moments['m00'])  # cx = M10/M00
            cy = int(self.moments['m01'] / self.moments['m00'])  # cy = M01/M00
            theta = 0.5 * np.arctan2(2 * self.moments['m11'],
                                     self.moments['m20'] - self.moments['m02'])
            theta = theta / np.pi * 180
        self.center_of_mass = (cx, cy)
        # calculate convexity defects
        self.defects = cv2.convexityDefects(self.cnt, self.hull)
        # evaluate defects two by two to get fingertips
        if self.defects is not None:
            for i in range(self.defects.shape[0]):
                s, e, f, d = self.defects[i, 0]
                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                far = tuple(cnt[f][0])
                if self.debug:
                    cv2.line(self.debug_img, start, end, [0, 0, 255], 1)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.circle(self.debug_img, self.center_of_mass, 2, [100, 0, 255],
                            -1)
        return self.center_of_mass
