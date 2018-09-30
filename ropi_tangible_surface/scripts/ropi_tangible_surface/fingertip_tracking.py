#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2
import numpy.ma as ma
import copy
import itertools
from enum import Enum
from ropi_msgs.msg import MultiTouch, SingleTouch
from ropi_msgs.srv import *
from geometry_msgs.msg import Point
from ropi_tangible_surface.base_classes import *
import uuid
euclidean_dist = lambda pt1, pt2: np.abs(np.linalg.norm(np.array(pt1) - np.array(pt2)))
ndargmin = lambda arr: np.unravel_index(arr.argmin(), arr.shape)

CursorState = {'PRESSED': 0, 'DRAGGED': 1, 'RELEASED': 2, 'UNDETERMINED': 3}

class IDManager:
    '''
    Cursor id assigning
    '''
    def __init__(self):
        self.ids = []
        self.next_id = 0
        self.poped_id = 0
    
    def new_id(self):
        self.ids.append(self.next_id)
        self.next_id = self.next_valid(self.next_id)
        return self.ids[-1]

    def release_id(self, id):
        if id in self.ids:
            self.ids.remove(id)
            self.next_id = min(id, self.next_id)
            return True
        else:
            return False

    def next_valid(self, id):
        while (id in self.ids):
            id += 1
        return id

DEBUG = False

class TouchTracker(TrackerBase):
    def __init__(self, pt, id=None):
        super(TouchTracker, self).__init__()
        # self.id = id
        self.position = pt
        self.position_prev = pt
        self.last_time = 0
        self.time = rospy.get_time()
        self.interval = 0
        self.state = CursorState['PRESSED']
        self.release_cnt = 0
        self.smoothing_factor = 0

    def elapsed_time(self):
        if self.last_time != 0:
            elapsed = rospy.get_time() - self.time
            # self.last_time = rospy.get_time()
            return elapsed
        else:
            return 0

    def update(self, pos):
        if pos is not None:
            # clear release cnt
            self.release_cnt = 0
            # time each update
            self.last_time = self.time
            self.time = rospy.get_time()
            self.interval = self.time - self.last_time
            # print(self.id,self.time-self.last_time)
            # update touch point
            self.position_prev = self.position
            self.position = np.int0(np.array(self.position) * self.smoothing_factor + (1 - self.smoothing_factor) * np.array(pos))
            self.displacement = euclidean_dist(self.position, self.position_prev)
            # self.position = pos
            self.state = CursorState['DRAGGED']
        else:
            self.time = rospy.get_time()
            self.interval = self.time - self.last_time
            # print(self.interval)
            if self.release_cnt > 5:#or self.interval >= 0.1:
                self.state = CursorState['UNDETERMINED']
                print('RELESE INTERVAL', self.interval)
            self.release_cnt += 1

    # TODO: write a service for this
    def is_released(self):
        if self.release_cnt > 10:
        # if self.interval >= 0.45:
            return True
        else:
            return False

    def make_msg(self):
        msg = SingleTouch()
        msg.id = self.id.hex
        msg.state = self.state
        msg.elapsed_time = self.elapsed_time()
        if DEBUG: print('pos: ', self.position)
        msg.cursor = Point(self.position[0], self.position[1], 0)
        msg.cursor_prev = Point(self.position_prev[0], self.position_prev[1], 0)
        return msg

class TouchTrackerManager(TrackingManagerBase):
    def __init__(self, screen_shape):
        rospy.wait_for_service('delete_cursor')
        self.delete_cursor = rospy.ServiceProxy('delete_cursor', DeleteSelection)
        self.width, self.height = screen_shape
        self.trackers = []
        self.move_threshold = 50
        # self.id_manager = IDManager()
        

    def update(self, pts):
        if DEBUG: print('update', pts)
        if len(self.trackers) == 0:
            self.new_trackers(pts)
        else: 
            self.match_trackers(pts)
            # print(self.trackers)

    def new_trackers(self, pts):
        for pt in pts:
            if not ma.is_masked(pt):
                new_cursor = TouchTracker(pt)
                self.trackers.append(new_cursor)
                if DEBUG: print('add pt', new_cursor.id.hex)

    def match_trackers(self, pts):
        # when only one finger, allow more tolerence
        if len(pts) == 1:
            move_threshold = 65
        else:
            move_threshold = self.move_threshold

        dists = [[euclidean_dist(cur.position, pt) for pt in pts] for cur in self.trackers]
        dists = ma.masked_greater(dists, move_threshold)
        unmatched_pts = ma.array(pts)
        processed_curs = []
        while unmatched_pts.compressed().size > 0 and dists.compressed().size > 0:
            arg_cur, arg_pt = ndargmin(dists)
            if DEBUG: print(dists)
            dists[(arg_cur, arg_pt)] = ma.masked
            unmatched_pts[arg_pt] = ma.masked
            if not arg_cur in processed_curs:
                processed_curs.append(arg_cur)
                self.trackers[arg_cur].update(pts[arg_pt])
            # TODO: fix this !!
        if DEBUG: print('Processed: ', processed_curs)
        # add new trackers
        if DEBUG: print('pts: ', pts)
        if DEBUG: print('unmatched: ', unmatched_pts)
        self.new_trackers(unmatched_pts)

        # clear released trackers
        for i, cur in enumerate(self.trackers):
            if not i in processed_curs:
                cur.update(None)
                if cur.is_released():
                    self.release_tracker(cur)

    def release_tracker(self, cur):
        try:
            print('---------Cursor release-----------')
            self.delete_cursor = rospy.ServiceProxy('delete_cursor', DeleteSelection)
            self.delete_cursor(cur.id.hex)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        cur.state = CursorState['RELEASED']
        self.trackers.remove(cur)
        # if self.id_manager.release_id(cur.id):
        #     self.trackers.remove(cur)
        # else:
        #     if DEBUG: print('Cursor id release unsuccessful!!')

    def make_msg(self):
        msg = MultiTouch()
        msg.width = self.width
        msg.height = self.height
        msg.header.stamp = rospy.Time.now()
        msg.cursors = []
        for cur in self.trackers:
            msg.cursors.append(cur.make_msg())
        return msg


        

