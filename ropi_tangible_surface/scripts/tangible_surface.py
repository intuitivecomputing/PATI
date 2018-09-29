#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import copy

import cv2

import rospy
import tf
import rospkg
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

from ropi_msgs.msg import *
from ropi_msgs.srv import *

from ropi_tangible_surface.transform import four_point_transform
from ropi_tangible_surface.fingertip_detection import *
from ropi_tangible_surface.fingertip_tracking import *
from ropi_tangible_surface.object_detection import *
from ropi_tangible_surface.object_tracking import *

from ropi_tangible_surface.selection import *
from pick_and_place import *



my_threshold = lambda img, l, u: (img < u) * (img > l) * img
flip_array = lambda a: np.hstack((np.hsplit(a, 2)[1], np.hsplit(a, 2)[0]))
tip_angle = lambda tip_pt, pt1, pt2: np.arctan2(pt2[1] - tip_pt[1], pt2[0] - tip_pt[0]) - np.arctan2(pt1[1] - tip_pt[1], pt1[0] - tip_pt[0])
euclidean_dist = lambda pt1, pt2: np.linalg.norm(np.array(pt1) - np.array(pt2))

class TangibleSurface:
    def __init__(self, resolution = (800, 450), use_skin_color_filter=True):
        # Constants and params
        self.resolution = resolution
        # self.aspect_ratio = resolution[0] / resolution[1]
        self.use_skin_color_filter = use_skin_color_filter
        self.load_data()
        self.on_init()
        self.ur5_init()

    def load_data(self):
        self.root_path = rospkg.RosPack().get_path('ropi_tangible_surface')
        self.ref_pts = np.load(self.root_path + '/config/points.npy')
        rospy.loginfo('Load reference points: ' + repr(np.asarray([self.ref_pts])))
        self.depth_background = np.load(self.root_path + '/config/depth.npy')

    def on_init(self):
        # Instances
        self.bridge = CvBridge()
        self.touch_detections = [FingertipDetection()] * 2
        self.touch_tracker_manager = TouchTrackerManager(self.resolution)

        self.object_detector = ObjectManager()
        self.object_tracker_manager = ObjectTrackerManager(self.resolution)
        
        self.selection_manager = SelectionManager(self.resolution)

        # publish amd subscribe
        self.finger_pub = rospy.Publisher("touch", MultiTouch, queue_size=50)
        self.skin_pub = rospy.Publisher("skin", Image, queue_size=50)
        self.debug_pub = rospy.Publisher("debug", Image, queue_size=50)
        self.obj_pub = rospy.Publisher("obj", Image, queue_size=50)
        self.subscribe('/kinect2/sd/image_color_rect',
                       '/kinect2/sd/image_depth_rect')

    def subscribe(self, img_topic, depth_topic):
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(img_topic, Image)
        self.ts = message_filters.TimeSynchronizer([depth_sub, rgb_sub], 20)
        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [depth_sub, rgb_sub], 5, 0.0012)
        self.ts.registerCallback(self.image_callback)
        self.region_selection_server = rospy.Service(
            'region_selection', RegionSelection, self.region_selection_callback)
        self.delete_selection_server = rospy.Service(
            'delete_selection', DeleteSelection, self.delete_selection_callback)
        self.move_server = rospy.Service(
            'move_objects', MoveObjects, self.move_objects_callback)
        self.grasp_pub = rospy.Publisher('grasp_data', GraspData, queue_size=1)

    def ur5_init(self):
        self.robot_interface = PickNPlace()

    def detect_objects_in_region(self, msg):
        self.selection_manager.update([msg])
        region = self.selection_manager.selections.get(msg.guid)
        rect = region.normalized_rect
        grasp_points = self.object_detector.get_grasp_selected(rect)
        return grasp_points

    def move_objects_callback(self, req):
        # TODO: finish this
        rospy.loginfo("move objects service called.")
        response = MoveObjectsResponse()
        mission = self.mission_from_regions(req.source_selection, req.target_selection)
        # if there are objects in the region
        if mission is not None:
            response.success = True
            response.message = '{} source objects detected.'.format(len(mission))
            rospy.loginfo('mission generated, executing mission.')
            # response.object_data = [x.make_msg() for x in mission]
            # self.grasp_pub.publish([x.make_msg() for x in mission])
            try:
                self.robot_interface.pick_and_place_mission(mission)
            except:
                rospy.logerr('mission failed.')
                response.success = False
                response.message = 'mission failed'
                return response
        else:
            response.success = False
            response.message = 'No source objects detected.'

        return response

    def place_position(self, pick_pos, source, target):
        relative_pos = np.asarray(pick_pos) - source.normalized_rect.get_center()
        scale = target.normalized_rect.diameter / source.normalized_rect.diameter
        if scale > 1:
            relative_pos = relative_pos * scale
        return target.normalized_rect.get_center() + relative_pos

    def mission_from_regions(self, source_region, target_region):
        print('Source: ', source_region)
        print('Target: ', target_region)
        grasp_points = self.detect_objects_in_region(source_region)
        print('Obj: ', grasp_points)
        if len(grasp_points) > 0:
            self.selection_manager.update([target_region])
            target = self.selection_manager.selections.get(target_region.guid)
            source = self.selection_manager.selections.get(source_region.guid)
            # print('centes: ', source.normalized_rect.get_center())
            # print('centes: ', target.normalized_rect.get_center())
            # movement = target.normalized_rect.get_center() - source.normalized_rect.get_center()
            # print('Displacement: {}'.format(movement))
            for g in grasp_points:
                g.target_position = np.int0(self.place_position(g.position, source, target))
            print('Mission: {}'.format(grasp_points))
            return grasp_points
        else:
            return None

    def delete_selection_callback(self, req):
        rospy.loginfo("Delete selection service called.")
        print (self.selection_manager.selections)
        self.selection_manager.delete([req.guid])
        print (self.selection_manager.selections)
        response = DeleteSelectionResponse()
        response.success = True
        response.message = 'Selection deleted.'
        return response

    def region_selection_callback(self, req):
        rospy.loginfo("Region selection service called.")
        grasp_points = self.detect_objects_in_region(req)

        response = RegionSelectionResponse()
        rospy.loginfo('Contructing response.')
        response.success = (len(grasp_points) > 0)
        response.message = repr(len(grasp_points)) + ' Obejcts found.'
        if response.success:
            grasp_data = []
            for gp in grasp_points:
                grasp_datum = GraspData()
                grasp_datum.position.x = gp.position[0]
                grasp_datum.position.y = gp.position[1]
                grasp_datum.diameter = gp.diameter 
                grasp_datum.angle = gp.angle
                grasp_datum.height = gp.height
                grasp_data.append(grasp_datum)
            response.grasp_data = grasp_data
        return response

    def image_callback(self, depth_in, rgb_in):
        cv_rgb = self.rgb_callback(rgb_in)
        cv_depth = self.depth_callback(depth_in)
        depth_foreground = self.depth_background - cv_depth
        # depth_foreground = self.filter(depth_foreground, size=3)
        depth_foreground[depth_foreground < 5] = 0
        # mask = my_threshold(depth_foreground, 50, 300))
        if (self.use_skin_color_filter):
            skin_mask = self.filter_skin(cv_rgb)
            skin = cv2.bitwise_and(
                depth_foreground, depth_foreground, mask=skin_mask)
            objects = cv2.bitwise_and(
                depth_foreground, depth_foreground, mask=~skin_mask)
        else:
            skin = depth_foreground.copy()
            objects = depth_foreground.copy()
        # warpped depth & rgb image for objects detection
        object_rgb_warpped = four_point_transform(cv_rgb, self.ref_pts)
        objects_warped = four_point_transform(objects, self.ref_pts)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(object_rgb_warpped))
        # self.selection_manager.update_image(objects_warped, object_rgb_warpped)
        objects = self.detect_object(objects_warped)
        self.object_tracker_manager.update(objects)
        # earpprd depth image for fingertip detection
        depth_warped = four_point_transform(skin, self.ref_pts)
        points = self.detect_fingertip(depth_warped)
        self.touch_tracker_manager.update(points)

        # print(self.touch_tracker_manager.make_msg())
        self.finger_pub.publish(self.touch_tracker_manager.make_msg())


    def rgb_callback(self, data):
        try:
            cv_image = copy.copy(self.bridge.imgmsg_to_cv2(data))
            return cv_image
        except CvBridgeError as e:
            print(e)

    def filter_skin(self, image):
        converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([0, 45, 60], dtype="uint8")
        upper = np.array([20, 150, 255], dtype="uint8")
        #lower = np.array([119, 59, 37], dtype = "uint8")
        #upper = np.array([150, 255, 255], dtype = "uint8")
        skin_mask = cv2.inRange(converted, lower, upper)
        skin_mask = self.filter(skin_mask)
        # skin = cv2.bitwise_and(cv_image, cv_image, mask=skin_mask)
        # self.skin_pub.publish(self.bridge.cv2_to_imgmsg(skin_mask))
        return skin_mask

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

    def filter(self, mask, size=3):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
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
        threshed = my_threshold(img, 0, 500)
        mask = np.zeros(img.shape, dtype=np.uint8)
        mask[threshed > 0] = 255
        dst = img.copy()
        mask = self.filter(mask)
        dst[~mask.astype(np.bool)] = 0
        contours = self.find_contour(mask)
        p = []
        merge_list = lambda l1, l2: l2 if not l1 else (l1 if not l2 else np.concatenate((l1, l2)))
        object_contours = []
        hand_contours = []
        if len(contours) != 0:
            contours = filter(lambda c: cv2.contourArea(c) > 200, contours)
            contours = np.asarray(sorted(
                contours, key=lambda cnt: cv2.contourArea(cnt), reverse=True))
            # filter out objects (hands' contour has nodes at edge)
            hand_candidate_contours = list(
                filter(lambda cnt: (True in [x[0][1] < 5 for x in cnt] or True in [x[0][0] < 5 for x in cnt]),
                       contours))
            hand_contours = np.asarray(hand_candidate_contours[0:2])
            for cnt in contours:
                if not np.any(hand_contours==cnt):
                    object_contours.append(cnt)
            debug_img = dst.copy()
            debug_img = cv2.cvtColor(
                debug_img.astype(np.uint8)[:, :, np.newaxis], cv2.COLOR_GRAY2BGR)
            # obj_debug_img = dst.copy()
            for i, cnt in enumerate(hand_candidate_contours[0:2]):
                p_new = self.touch_detections[i].update(cnt, dst, debug_img)
                p = merge_list(p, p_new)
                debug_img = self.touch_detections[i].debug_img
            if len(p) != 0: 
                rospy.loginfo(repr(len(p)) + ' touch points: ' + repr(p))
            self.skin_pub.publish(self.bridge.cv2_to_imgmsg(debug_img))
            

        return p
    
    def detect_object(self, img):
        objects = []
        threshed = my_threshold(img, 5, 150)
        mask = np.zeros(img.shape, dtype=np.uint8)
        mask[threshed > 0] = 255
        dst = img.copy()
        mask = self.filter(mask)
        dst[~mask.astype(np.bool)] = 0
        contours = self.find_contour(mask)
        merge_list = lambda l1, l2: l2 if not l1 else (l1 if not l2 else np.concatenate((l1, l2)))
        object_contours = []
        if len(contours) != 0:
            # print([cv2.contourArea(c) for c in contours])
            contours = filter(lambda c: cv2.contourArea(c) > 400 and cv2.contourArea(c) < 3000, contours)
            debug_img = dst.copy()
            objects = self.object_detector.update(contours, dst, debug_img)
            debug_img = self.object_detector.debug_img
            debug_img = self.selection_manager.draw(debug_img)
            debug_img = self.object_detector.draw_selections(debug_img, self.selection_manager.get_rects())
            self.obj_pub.publish(self.bridge.cv2_to_imgmsg(debug_img))
        return objects


def main(args):
    rospy.init_node("gest")
    dp = TangibleSurface(use_skin_color_filter=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)