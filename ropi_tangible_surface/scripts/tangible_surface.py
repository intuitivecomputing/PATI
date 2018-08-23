#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import rospkg
import message_filters
import copy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ropi_tangible_surface.transform import four_point_transform
from ropi_tangible_surface.fingertip_detection import *
from ropi_tangible_surface.fingertip_tracking import *
import cv2
from ropi_msgs.msg import MultiTouch, SingleTouch
from geometry_msgs.msg import Point

my_threshold = lambda img, l, u: (img < u) * (img > l) * img
flip_array = lambda a: np.hstack((np.hsplit(a, 2)[1], np.hsplit(a, 2)[0]))
tip_angle = lambda tip_pt, pt1, pt2: np.arctan2(pt2[1] - tip_pt[1], pt2[0] - tip_pt[0]) - np.arctan2(pt1[1] - tip_pt[1], pt1[0] - tip_pt[0])
euclidean_dist = lambda pt1, pt2: np.linalg.norm(np.array(pt1) - np.array(pt2))


class TangibleSurface:
    def __init__(self, use_skin_color_filter=True):
        self.use_skin_color_filter = use_skin_color_filter
        rospack = rospkg.RosPack()
        self.root_path = rospack.get_path('ropi_tangible_surface')
        self.depth_background = np.load(self.root_path + '/config/depth.npy')

        self.bridge = CvBridge()
        self.detections = [FingertipDetection()] * 2
        self.finger_pub = rospy.Publisher("touch", MultiTouch, queue_size=50)
        self.skin_pub = rospy.Publisher("skin", Image, queue_size=50)
        self.ref_pts = np.load(self.root_path + '/config/points.npy')
        print(np.asarray([self.ref_pts]))
        # self.rgb_ref_pts = np.array(
        #     [(81, 90), (30, 275), (458, 252), (402, 73)], dtype="float32")
        # self.depth_ref_pts = np.array([[(75, 88), (26, 276), (460, 253),
        #                                 (397, 76)]])
        self.tracker_manager = TrackerManager((450, 800))
        self.subscribe('/kinect2/sd/image_color_rect', '/kinect2/sd/image_depth_rect')

    def subscribe(self, img_topic, depth_topic):
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(img_topic, Image)
        self.ts = message_filters.TimeSynchronizer([depth_sub, rgb_sub], 20)
        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [depth_sub, rgb_sub], 5, 0.0012)
        self.ts.registerCallback(self.image_callback)
        
    def image_callback(self, depth_in, rgb_in):
        skin_mask = self.rgb_callback(rgb_in)
        cv_depth = self.depth_callback(depth_in)
        depth_foreground = self.depth_background - cv_depth
        depth_foreground[depth_foreground < 5] = 0
        # mask = my_threshold(depth_foreground, 50, 300))
        if (self.use_skin_color_filter):
            skin = cv2.bitwise_and(
                depth_foreground, depth_foreground, mask=skin_mask)
        else:
            skin = depth_foreground.copy()
        warped_depth = four_point_transform(skin, self.ref_pts)
        points = self.detect_fingertip(warped_depth)
        touch_points = []
        r, c = warped_depth.shape
        for p in points:
            # print(r, c, p[1], p[0])
            np = Point(p[0] / c, p[1] / r, 0)
            touch_points.append(np)
        # print('points: ' , points)
        # print('cursors: ', self.tracker_manager.cursors)
        self.tracker_manager.update(points)
        print(self.tracker_manager.make_msg())
        self.finger_pub.publish(self.tracker_manager.make_msg())

    def rgb_callback(self, data):
        try:
            cv_image = copy.copy(self.bridge.imgmsg_to_cv2(data))
            converted = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            lower = np.array([0, 45, 60], dtype="uint8")
            upper = np.array([20, 150, 255], dtype="uint8")
            #lower = np.array([119, 59, 37], dtype = "uint8")
            #upper = np.array([150, 255, 255], dtype = "uint8")
            skin_mask = cv2.inRange(converted, lower, upper)
            skin_mask = self.filter(skin_mask)
            # skin = cv2.bitwise_and(cv_image, cv_image, mask=skin_mask)
            # self.skin_pub.publish(self.bridge.cv2_to_imgmsg(skin_mask))
            return skin_mask
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            mask = self.make_mask(np.asarray([self.ref_pts]), cv_image.shape)
            masked_image = copy.copy(cv_image)
            masked_image[np.isnan(masked_image)] = 0
            masked_image[~mask] = 0
            return masked_image
        except CvBridgeError as e:
            print(e)

    def filter(self, mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.GaussianBlur(mask, (3, 3), 0)
        return mask

    def find_contour(self, mask):
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        return contours

    def make_mask(self, pts, shape):
        roi_corners = pts.astype(np.int32)
        mask = np.zeros(shape, dtype=np.uint8)[:, :, np.newaxis]
        cv2.fillPoly(mask, np.array([roi_corners]), (255, ))
        mask = mask.astype(np.bool)
        mask = np.resize(mask, shape)
        return mask

    def detect_fingertip(self, img):
        threshed = my_threshold(img, 0, 300)
        mask = np.zeros(img.shape, dtype=np.uint8)
        mask[threshed > 0] = 255
        dst = img.copy()
        mask = self.filter(mask)
        dst[~mask.astype(np.bool)] = 0
        contours = self.find_contour(mask)
        p = []
        if len(contours) != 0:
            contours = sorted(
                contours, key=lambda cnt: cv2.contourArea(cnt), reverse=True)

            p = self.detections[0].update(contours[0], dst, dst)
            if len(contours) > 1:
                p = p + self.detections[1].update(contours[1], dst,
                                                  self.detections[0].debug_img)
                debug_img = self.detections[1].debug_img
            else:
                debug_img = self.detections[0].debug_img
            # for i, cnt in enumerate(contours[0:2]):
            #     self.detections[i].update(cnt, dst, debug_img)
            # debug_img = self.detections[i].debug_img
            # print(p)
            self.skin_pub.publish(self.bridge.cv2_to_imgmsg(debug_img))
        self.detections[0].debug_img = None
        self.detections[1].debug_img = None

        return p

def main(args):
    rospy.init_node("gest")
    dp = TangibleSurface(use_skin_color_filter=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)