#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2
import numpy.ma as ma
import copy
import itertools
from enum import Enum
from ropi_msgs.msg import MultiTouch, SingleTouch
from geometry_msgs.msg import Point

euclidean_dist = lambda pt1, pt2: np.abs(np.linalg.norm(np.array(pt1) - np.array(pt2)))
ndargmin = lambda arr: np.unravel_index(arr.argmin(), arr.shape)

CursorState = {'PRESSED': 0, 'DRAGGED': 1, 'RELEASED': 2}

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



class TouchTracker:
    def __init__(self, id, pt):
        self.id = id
        self.position = pt
        self.position_prev = pt
        self.last_time = 0
        self.time = rospy.get_time()
        self.state = CursorState['PRESSED']
        self.release_cnt = 0
        self.smoothing_factor = 0.2

    def elapsed_time(self):
        if self.last_time != 0:
            return self.time - self.last_time
        else:
            return 0

    def update(self, pos):
        if pos is not None:
            # clear release cnt
            self.release_cnt = 0
            # time each update
            self.last_time = self.time
            self.time = rospy.get_time()
            # update touch point
            self.position_prev = self.position
            self.position = np.int0(np.array(self.position) * self.smoothing_factor + (1 - self.smoothing_factor) * np.array(pos))
            # self.position = pos
            self.state = CursorState['DRAGGED']
        else:
            if self.release_cnt > 10:
                self.state = CursorState['RELEASED']
            self.release_cnt += 1

    def is_released(self):
        if self.release_cnt > 15:
            return True
        else:
            return False

    def make_msg(self):
        msg = SingleTouch()
        msg.id = self.id
        msg.state = self.state
        msg.elapsed_time = self.elapsed_time()
        print('pos: ', self.position)
        msg.cursor = Point(self.position[0], self.position[1], 0)
        msg.cursor_prev = Point(self.position_prev[0], self.position_prev[1], 0)
        return msg

class TrackerManager:
    def __init__(self, screen_shape):
        self.height, self.width = screen_shape
        self.cursors = []
        self.move_threshold = 10
        self.id_manager = IDManager()

    def update(self, pts):
        print('update', pts)
        if len(self.cursors) == 0:
            self.new_cursors(pts)
        else: 
            self.match_cursors(pts)

    def new_cursors(self, pts):
        for pt in pts:
            if not ma.is_masked(pt):
                new_cursor = TouchTracker(self.id_manager.new_id(), pt)
                self.cursors.append(new_cursor)
                print('add pt', new_cursor.id)

    def match_cursors(self, pts):
        dists = [[euclidean_dist(cur.position, pt) for pt in pts] for cur in self.cursors]
        dists = ma.masked_greater(dists, self.move_threshold)
        unmatched_pts = ma.array(pts)
        processed_curs = []
        while unmatched_pts.compressed().size > 0 and dists.compressed().size > 0:
            arg_cur, arg_pt = ndargmin(dists)
            print(dists)
            dists[(arg_cur, arg_pt)] = ma.masked
            unmatched_pts[arg_pt] = ma.masked
            if not arg_cur in processed_curs:
                processed_curs.append(arg_cur)
                self.cursors[arg_cur].update(pts[arg_pt])
            # TODO: fix this !!
        print('Processed: ', processed_curs)

        # add new cursors
        print('pts: ', pts)
        print('unmatched: ', unmatched_pts)
        self.new_cursors(unmatched_pts)

        # clear released cursors
        for i, cur in enumerate(self.cursors):
            if not i in processed_curs:
                cur.update(None)
                if cur.is_released():
                    self.release_cursor(cur)

    def release_cursor(self, cur):
        if self.id_manager.release_id(cur.id):
            self.cursors.remove(cur)
        else:
            print('Cursor id release unsuccessful!!')

    def make_msg(self):
        msg = MultiTouch()
        msg.width = self.width
        msg.height = self.height
        msg.header.stamp = rospy.Time.now()
        msg.cursors = []
        for cur in self.cursors:
            msg.cursors.append(cur.make_msg())
        return msg


        

