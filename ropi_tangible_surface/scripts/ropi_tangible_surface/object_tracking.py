#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

from ropi_tangible_surface.fingertip_tracking import *
from ropi_tangible_surface.selection import *
from ropi_msgs.msg import SingleObject, MultiObject
import rospy
import cv2

ObjectState = {'DETECTED': 0, 'MOVED': 1, 'LOST': 2}

DEBUG = False


class ObjectTracker(TouchTracker):
    # TYPE = {TYPE_OBJECT_SELECTION: 0, TYPE_AREA_SELECTION: 1}
    def __init__(self, msg):
        super(ObjectTracker, self).__init__(self)
        self.position = msg.position
        self.position_prev = msg.position
        self.state = ObjectState['DETECTED']
        self.angle = msg.angle
        self.diameter = msg.diameter
        self.height = msg.height
        self.smoothing_factor = 0.2
        self.color = np.random.randint(0,255,(3)).tolist()

    def update(self, data ):
        smoothing_filter = lambda x1, x2, a: x1 * a + x2 * (1 - a)
        if data is not None:
            # clear release cnt
            self.release_cnt = 0
            # time each update
            self.last_time = self.time
            self.time = rospy.get_time()
            # update touch point
            self.position_prev = self.position
            self.position = np.int0(np.array(self.position) * self.smoothing_factor + (1 - self.smoothing_factor) * np.array(data.position))
            # self.position = pos
            self.state = ObjectState['MOVED']
            self.angle = smoothing_filter(self.angle, data.angle, self.smoothing_factor)
            self.diameter = smoothing_filter(self.diameter, data.diameter, self.smoothing_factor)
            self.height = smoothing_filter(self.height, data.height, self.smoothing_factor)
        else:
            if self.release_cnt > 10:
                self.state = ObjectState['LOST']
            self.release_cnt += 1

    def make_msg(self):
        msg = SingleObject()
        msg.id = self.id.hex
        msg.state = self.state
        msg.elapsed_time = self.elapsed_time()
        msg.angle = self. angle
        msg.height = self.height
        msg.diameter = self.diameter
        if DEBUG: print('pos: ', self.position)
        msg.position = Point(self.position[0], self.position[1], 0)
        msg.position_prev = Point(self.position_prev[0], self.position_prev[1], 0)
        return msg 

class ObjectTrackerManager(TouchTrackerManager):
    def __init__(self, res = (800, 450)):
        self.width, self.height = res
        self.trackers = []
        self.move_threshold = 50

    def new_trackers(self, objs):
        for obj in objs:
            if not ma.is_masked(obj):
                new_object = ObjectTracker(obj)
                self.trackers.append(new_object)
                if DEBUG: print('add obj', new_object.id.hex)


    def match_trackers(self, objs):
        dists = [[euclidean_dist(cur.position, obj.position) for obj in objs] for cur in self.trackers]
        dists = ma.masked_greater(dists, self.move_threshold)
        unmatched_objs = ma.array(objs)
        # unmatched_objs = ma.array(map(lambda x: x.position, objs))
        processed_curs = []
        while unmatched_objs.compressed().size > 0 and dists.compressed().size > 0:
            arg_cur, arg_obj = ndargmin(dists)
            if DEBUG: print(dists)
            dists[(arg_cur, arg_obj)] = ma.masked
            unmatched_objs[arg_obj] = ma.masked
            if not arg_cur in processed_curs:
                processed_curs.append(arg_cur)
                self.trackers[arg_cur].update(objs[arg_obj])
            # TODO: fix this !!
        if DEBUG: print('Processed: ', processed_curs)

        # add new trackers
        if DEBUG: print('objs: ', objs)
        if DEBUG: print('unmatched: ', unmatched_objs)
        self.new_trackers(unmatched_objs)

        # clear released trackers
        for i, cur in enumerate(self.trackers):
            if not i in processed_curs:
                cur.update(None)
                if cur.is_released():
                    self.release_tracker(cur)


    def make_msg(self):
        msg = MultiObject()
        msg.width = self.width
        msg.height = self.height
        msg.header.stamp = rospy.Time.now()
        msg.objects = map(lambda obj: obj.make_msg(), self.trackers)
        # for obj in self.trackers:
        #     msg.objects.append(obj.make_msg())
        return msg