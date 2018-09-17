#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import rospkg
import message_filters
import copy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ropi_tangible_surface.transform import four_point_transform
import numpy as np
import math
import cv2
import argparse

my_threshold = lambda img, l, u: (img < u) * (img > l) * img
flip_array = lambda a: np.hstack((np.hsplit(a, 2)[1], np.hsplit(a, 2)[0]))
tip_angle = lambda tip_pt, pt1, pt2: np.arctan2(pt2[1] - tip_pt[1], pt2[0] - tip_pt[0]) - np.arctan2(pt1[1] - tip_pt[1], pt1[0] - tip_pt[0])
euclidean_dist = lambda pt1, pt2: np.linalg.norm(np.array(pt1) - np.array(pt2))

SHOW = True
DRAW = True

def show_depth(image, name='Image'):
    if SHOW:
        image_norm = np.zeros(image.shape, dtype=np.float32)
        # print (np.max(image))
        cv2.normalize(
            image.astype(np.float32), image_norm, 0, 1, cv2.NORM_MINMAX)
        # print (np.max(image_norm))
        cv2.imshow(name, image_norm)
        cv2.waitKey(1)


def show(image, name='Image'):
    if SHOW:
        cv2.imshow(name, image)
        cv2.waitKey(1)



class CalibrateWorkspace:
    def __init__(self, rgb_topic, depth_topic, calib_rgb=False):
        self.bridge = CvBridge()
        
        self.image_rgb = None
        self.image_depth = None
        self.ref_pts = []
        self.run_rgb_calib = calib_rgb
        self.run_depth_calib = not calib_rgb
        rospack = rospkg.RosPack()
        self.root_path = rospack.get_path('ropi_tangible_surface')

        self.depth_cnt = 0
        rospy.Subscriber(depth_topic, Image, self.depth_callback)
        rospy.Subscriber(rgb_topic, Image, self.rgb_callback)


    def click_and_crop(self, event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            self.ref_pts.append((x, y))
            # cv2.circle(self.image_rgb, (x, y), 2, (255, 0, 0), 2)
            if len(self.ref_pts) == 4:
                print(self.ref_pts)

    def rgb_callback(self, data):
        try:
            if self.run_rgb_calib:
                cv_rgb = self.bridge.imgmsg_to_cv2(data)
                calib_image = cv_rgb.copy()
            
                cv2.namedWindow("calib_image")
                cv2.setMouseCallback("calib_image", self.click_and_crop)
                while True:
                    # display the image and wait for a keypress
                    cv2.imshow("calib_image", calib_image)
                    key = cv2.waitKey(1) & 0xFF

                    # if the 'r' key is pressed, reset the cropping region
                    if key == ord("r"):
                        calib_image = cv_rgb.copy()

                    # if the 'c' key is pressed, break from the loop
                    elif key == ord("c"):
                        break
                pts = np.array(self.ref_pts, dtype = "float32")
                np.save(self.root_path + '/config/points.npy', pts)
                print('Points saved')
                # apply the four point tranform to obtain a "birds eye view" of
                # the image
                warped = four_point_transform(cv_rgb, pts)


                # show the original and warped images
                cv2.imshow("Original", cv_rgb)
                cv2.imshow("Warped", warped)
                cv2.waitKey(0)
                self.run_rgb_calib = False
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            if self.run_depth_calib:
                cv_depth = self.bridge.imgmsg_to_cv2(data)
            
                print("calib depth")
                
                if self.depth_cnt == 0:
                    self.depth_pre = np.zeros(cv_depth.shape, dtype='uint32')
                if self.depth_cnt < 20:
                    self.depth_pre = self.depth_pre + cv_depth
                    self.depth_cnt += 1
                else:
                    self.depth_pre.astype('float32')
                    self.depth_pre = self.depth_pre / 20.0
                    
                    np.save(self.root_path + '/config/depth.npy', self.depth_pre)
                    print('Depth premitive saved')
                    show_depth(self.depth_pre, 'depth')
                    pts = np.load(self.root_path + '/config/points.npy')
                    warped = four_point_transform(self.depth_pre, pts)
                    print(warped.shape)
                    show_depth(warped, 'depth_warped')
                    cv2.waitKey(0)
                    self.run_depth_calib = False
                # show_depth(self.depth_pre, 'depth')
                # cv2.setMouseCallback('depth', self.click_and_crop)
                # while True:
                #     # display the image and wait for a keypress
                #     # cv2.imshow('depth', self.depth_pre)
                #     key = cv2.waitKey(1) & 0xFF

                #     # if the 'c' key is pressed, break from the loop
                #     if key == ord("c"):
                #         break
                # pts = np.array(self.ref_pts, dtype = "float32")
                # np.save(self.root_path + '/config/depth_points.npy', pts)
                # print('Points saved')
                # # apply the four point tranform to obtain a "birds eye view" of
                # # the image
                # warped = four_point_transform(self.depth_pre, pts)


                # # show the original and warped images
                # cv2.imshow("Original", cv_depth)
                # cv2.imshow("Warped", warped)
                    
        except CvBridgeError as e:
            print(e)



def main(args):
    
    rospy.init_node("Calibration")
    calib = CalibrateWorkspace('/kinect2/sd/image_color_rect', '/kinect2/sd/image_depth_rect', calib_rgb=args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Callibration type.')
    parser.add_argument('--calib_rgb', metavar='T/F', type=bool, nargs='+',
                    help='Calib RGB or Depth')
    args = parser.parse_args()
    print (args)
    main(args.calib_rgb)