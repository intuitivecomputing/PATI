#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import cv2
import numpy.ma as ma
import copy
import uuid
import itertools
# from tabletop_proc.msg import MultiTouch, SingleTouch
from enum import Enum
from ropi_msgs.msg import *
from geometry_msgs.msg import Point

euclidean_dist = lambda pt1, pt2: np.abs(np.linalg.norm(np.array(pt1) - np.array(pt2)))
ndargmin = lambda arr: np.unravel_index(arr.argmin(), arr.shape)

class CursorState(Enum):
    CURSOR_PRESSED = 0
    CURSOR_DRAGGED = 1
    CURSOR_RELEASED = 2

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

    def remove_id(self, id):
        self.ids.remove(id)
        self.next_id = min(id, self.next_id)

    def next_valid(self, id):
        while (id in self.ids):
            id += 1
        return id



class TouchTracker:
    def __init__(self, id, pt):
        self.id = id
        self.position = pt
        self.position_prev = None
        self.state = CursorState.CURSOR_PRESSED
        self.release_cnt = 0
        
    def update(self, pos):
        if pos is not None:
            self.release_cnt = 0
            self.position_prev = self.position
            self.position = pos
            self.state = CursorState.CURSOR_DRAGGED
        else:
            if self.release_cnt > 10:
                self.state = CursorState.CURSOR_RELEASED
            self.release_cnt += 1

    def is_released(self):
        if self.state == CursorState.CURSOR_RELEASED:
            return True
        else:
            return False

    def make_msg(self):
        msg = SingleTouch()
        msg.id = self.id
        msg.state = self.state
        msg.cursor = Point(self.position[0], self.position[1], 0)
        msg.cursor_prev = Point(self.position_prev[0], self.position_prev[1], 0)
        return msg

class TrackerManager:
    def __init__(self, screen_shape):
        self.height, self.width = screen_shape
        self.cursors = []
        self.move_threshold = 0.5
        self.id_manager = IDManager()

    def update(self, pts):
        if self.cursors is None:
            for pt in pts:
                new_cursor = TouchTracker(self.id_manager.new_id(), pt)
                self.cursors.append(new_cursor)
        else: 
            self.match_cursors(pts)

    def match_cursors(self, pts):
        dists = [[euclidean_dist(cur.position, pt) for pt in pts] for cur in self.cursors]
        dists = ma.masked_greater(dists, self.move_threshold)
        processed_curs = []
        while dists.compressed().size > 0:
            arg_cur, arg_pt = dists[ndargmin(dists)]
            if not arg_cur in processed_curs:
                processed_curs.append(arg_cur)
                self.cursors[arg_cur].update(pts[arg_pt])

        for i, cur in enumerate(self.cursors):
            if not i in processed_curs:
                cur.update(None)
                if cur.is_released():
                    self.cursors.remove(cur)
    
    def make_msg(self):
        msg = MultiTouch()
        msg.width = self.width
        msg.height = self.height
        msgs.header.stamp = rospy.Time.now()
        msg.cursors = []
        for cur in self.cursors:
            msg.cursors.append(cur.make_msg())
        return msg


        

