#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2
import tf

from ropi_msgs.msg import GraspData
from geometry_msgs.msg import Point

class GraspDataClass:
    '''container class for objects' information'''
    def __init__(self, position, diameter, angle, height):
        self.position = np.asarray(position)
        self.target_position = None
        self.diameter = diameter / 1000
        self.angle = angle
        self.height = height
    
    def get_points(self):
        pt1 = self.position + self.diameter * np.array([np.sin(self.angle), np.cos(self.angle)])
        pt2 = self.position - self.diameter * np.array([np.sin(self.angle), np.cos(self.angle)])
        return tuple(np.int0(pt1)), tuple(np.int0(pt2))

    def draw(self, img, color):
        debug_img = img.copy()
        cv2.circle(debug_img, tuple(np.int0(self.position)), int(self.diameter), color, 5)
        pt1, pt2 = self.get_points()
        # print (pt1, pt2)
        cv2.line(debug_img, pt1, pt2, color, 5)
        return debug_img

    def make_msg(self):
        msg = GraspData()
        msg.position = Point(self.position[0], self.position[1], self.height)
        if self.target_position is not None:
            msg.target_position = Point(self.target_position[0], self.target_position[1], self.height)
        msg.diameter = self.diameter
        msg.height = self.height
        msg.angle = self.angle

    def __repr__(self):
        return '[GraspDataClass]: {}-{}-|Height: {}|Angle: {} |Diameter: {}'.format(self.position, self.target_position, self.height,self.angle, self.diameter)
        # return repr(self.position)

class ObjectManager:
    def __init__(self, debug = True):
        self.init()
        self.debug = debug
        self.br = tf.TransformBroadcaster()


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

    def update(self, cnts, depth_img, debug_img=None):
        '''update objects
        input: 
            cnts: list of contours
            depth_img: depth image for detection
            debug_img(optional): rgb img for visualization'''
        self.init()
        output = []
        if self.debug and debug_img is None:
            self.set_debug_img(depth_img)
        if debug_img is not None:
            self.set_debug_img(debug_img)
        self.depth_img = depth_img
        for cnt in cnts:
            detection = ObjectDetection()
            out = detection.update(cnt, self.depth_img, self.debug_img)
            self.detections.append(detection)
            output.append(out)

        
        return output

    def get_grasp_data(self):
        grasp_data = [od.get_grasp_data() for od in self.detections]
        return grasp_data

    def get_grasp_selected(self, rect):
        return [od.grasp_data for od in self.detections if od.inside(rect)]

    def draw_selections(self, img, rects):
        debug_img = img.copy()
        for rect in rects:
            color = np.random.randint(0,255,(3)).tolist()
            for gd in self.get_grasp_selected(rect):
                debug_img = gd.draw(debug_img, color)
        return debug_img

class ObjectDetection:
    '''object detection class'''
    def __init__(self, debug=True):
        self.clear()
        self.debug = debug

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
        self.object_height = None
        self.grasp_points = None
        self.depth_img = None
        self.debug_img = None

    def update(self, cnt, depth_img, debug_img=None):
        '''   
        make update
        inputs:
            cnt: contour of object
            depth_img: depth image for detection
            debug_img(optional): rgb img for visualization
        '''
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
        if self.debug: 
            cv2.circle(self.debug_img, self.center_of_mass, 2, [100, 0, 255],
                    -1)
        # calculate convexity defects
        # self.defects = cv2.convexityDefects(self.cnt, self.hull)
        # # evaluate defects two by two to get fingertips
        # if self.defects is not None:
        #     for i in range(self.defects.shape[0]):
        #         s, e, f, d = self.defects[i, 0]
        #         start = tuple(cnt[s][0])
        #         end = tuple(cnt[e][0])
        #         far = tuple(cnt[f][0])
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # print (rect[2])
        if self.debug:
            cv2.drawContours(self.debug_img, [box], 0, (0,0,255),2)

        ellipse = cv2.fitEllipse(cnt)

        self.grasp_points = self.get_major_axis(box)
        if self.debug: 
            cv2.ellipse(self.debug_img, ellipse,(0,255,0),2)
            cv2.line(self.debug_img, self.grasp_points[0], self.grasp_points[1], [0, 0, 255], 2)
        grasp_sorted = sorted([self.grasp_points[0], self.grasp_points[1]], key=lambda x: x[1])
        angle = 180. / np.pi * np.arctan2(grasp_sorted[1][1] - grasp_sorted[0][1], grasp_sorted[0][0] - grasp_sorted[1][0])
        # print(grasp_sorted[1][1] - grasp_sorted[0][1], grasp_sorted[0][0] - grasp_sorted[1][0])
        # print(grasp_sorted)
        mask = self.get_mask()
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(self.depth_img, mask = mask)  
        # from mm to m
        self.object_height = max_val / 1000
        # print(min_val, min_loc, max_val, max_loc)
        self.grasp_data = GraspDataClass(self.center_of_mass, self.euclidean_dist(box[0], box[2]), angle, self.object_height) # center, diameter, angle, height
        # print(self.grasp_data)
        return self.grasp_data


    @staticmethod
    def get_major_axis(rect):
        # (x, y), (MA, ma), angle = ellipse
        # major_axis = min(MA, ma)
        # # MA_x, MA_y = int(0.5 * MA * math.sin(angle)), int(0.5 * MA * math.cos(angle))
        # ma_x, ma_y = int(0.5 * major_axis * math.sin(angle)), int(0.5 * major_axis * math.cos(angle))
        # ma_x_top, ma_y_top = int(x + ma_x), int(y + ma_y)
        # ma_x_bot, ma_y_bot = int(x - ma_x), int(y - ma_y)
        # return (ma_x_top, ma_y_top), (ma_x_bot, ma_y_bot)
        axis = None
        rect = np.asarray(rect)
        axis1 = ((rect[0] + rect[1]) / 2, (rect[2] + rect[3]) / 2)
        axis2 = ((rect[0] + rect[3]) / 2, (rect[2] + rect[1]) / 2)
        axis1_length = ObjectDetection.euclidean_dist(axis1[0], axis1[1])
        axis2_length = ObjectDetection.euclidean_dist(axis2[0], axis2[1])
        if axis1_length >= axis2_length:
            axis = axis2
        else:
            axis = axis1
        axis = tuple(np.int0(axis[0])), tuple(np.int0(axis[1]))
        return axis

    def get_mask(self):
        mask = np.zeros(self.depth_img.shape, np.uint8)
        cv2.drawContours(mask, [self.cnt], 0, 255, -1)
        # pixel_points = np.transpose(np.nonzero(mask))
        return mask

    def get_grasp_data(self):
        return self.grasp_data

    def inside(self, rect):
        return rect.inside(self.center_of_mass)
