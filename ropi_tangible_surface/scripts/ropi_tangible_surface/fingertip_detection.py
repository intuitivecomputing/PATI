#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2


class FingertipDetection:
    def __init__(self, debug=True):
        self.cnt = None
        self.hull = None
        self.moments = None
        self.defects = None
        self.center_of_mass = None
        self.tip_points = []
        self.debug = debug
        self.depth_img = None
        self.debug_img = None
        self.window_size = 10

    @staticmethod
    def euclidean_dist(pt1, pt2):
        return np.linalg.norm(np.array(pt1) - np.array(pt2))

    @staticmethod
    def tip_angle(tip_pt, pt1, pt2):
        return np.arctan2(pt2[1] - tip_pt[1], pt2[0] - tip_pt[0]) - np.arctan2(
            pt1[1] - tip_pt[1], pt1[0] - tip_pt[0])

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
        self.tip_points = []
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
            # put first defect to the end
            self.defects = np.append(self.defects, [self.defects[0, :, :]], axis=0)
            for i in range(self.defects.shape[0] - 1):
                s, e, f, d = self.defects[i, 0, :]
                sn, en, fn, dn = self.defects[i + 1, 0, :]
                if self.depth_img[self.center_of_mass[1], self.center_of_mass[0]] > 30 and (d > 200 or dn > 200):
                    start = tuple(cnt[s][0])
                    end = tuple(cnt[e][0])
                    far = tuple(cnt[f][0])
                    startn = tuple(cnt[sn][0])
                    endn = tuple(cnt[en][0])
                    farn = tuple(cnt[fn][0])
                    # tip points is in the middle of end and startn
                    tip_pt = tuple(np.int0(cnt[e][0] * 0.5 + cnt[sn][0] * 0.5))
                    tip_dist = self.euclidean_dist(end, startn)
                    tip_angle = np.abs(
                        self.tip_angle(tip_pt, far, farn) / np.pi * 180)
                    # print('center: ', self.center_of_mass, ' ', self.depth_img[self.center_of_mass[1], self.center_of_mass[0]])
                    if tip_angle < 60 and tip_angle > 0 and tip_dist < 20 :
                        self.tip_points.append(tip_pt)
                        if self.debug:
                            cv2.line(self.debug_img, tip_pt, far, [0, 0, 255], 1)
                            cv2.line(self.debug_img, tip_pt, farn, [0, 0, 255], 1)
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            cv2.circle(self.debug_img, tip_pt, 2, [100, 0, 255],
                                    -1)
                            # cv2.putText(self.debug_img,'angle:' + str(angle),tip_pt, font, 0.5,(255,0,255),2,cv2.LINE_AA)
                    if self.debug:
                        cv2.circle(self.debug_img, self.center_of_mass, 2, [100, 0, 255],
                                    -1)
                        cv2.circle(self.debug_img, far, 2, [100, 0, 255], -1)
                        cv2.circle(self.debug_img, start, 2, [255, 0, 0], -1)
                        cv2.circle(self.debug_img, end, 2, [255, 0, 100], -1)
            self.tip_points = self.filter_tips(self.tip_points)
        return self.tip_points

    def filter_tips(self, tip_points):
        touch_points = []
        for i, tip in enumerate(tip_points):
            tip_window = self.depth_img[
                tip[1] - self.window_size:tip[1] + self.window_size, tip[0] -
                self.window_size:tip[0] + self.window_size]
            if tip_window.size > 0:
                tip_depth = tip_window.max()
                if self.debug:
                    # print('depth: ', tip_depth)
                    cv2.rectangle(
                        self.debug_img,
                        tuple(
                            np.array(tip) -
                            np.array([self.window_size, self.window_size])),
                        tuple(
                            np.array(tip) +
                            np.array([self.window_size, self.window_size])),
                        (255, 200, 200), 1)
                if tip_depth < 18:# and tip_depth > 0:
                    if self.debug:
                        cv2.circle(self.debug_img, tip, 5, [100, 0, 255], -1)
                    touch_points.append(tip)
        return touch_points
