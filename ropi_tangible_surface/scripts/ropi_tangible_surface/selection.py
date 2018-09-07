#!/usr/bin/env python
# py2/3 compatibility
from __future__ import (absolute_import, division, print_function,
                        unicode_literals)

from ropi_tangible_surface.common_imports import *
from ropi_tangible_surface.base_classes import *
from ropi_msgs.srv import *
import uuid
import itertools

import cv2
import rospy
DEBUG = False

def debug_log(*args):
    if DEBUG:
        print(*args)

class Rectangle:
    def intersection(self, other):
        a, b = self, other
        x1 = max(min(a.x1, a.x2), min(b.x1, b.x2))
        y1 = max(min(a.y1, a.y2), min(b.y1, b.y2))
        x2 = min(max(a.x1, a.x2), max(b.x1, b.x2))
        y2 = min(max(a.y1, a.y2), max(b.y1, b.y2))
        if x1 < x2 and y1 < y2:
            return type(self)(x1, y1, x2, y2)
    __and__ = intersection

    def difference(self, other):
        inter = self & other
        if not inter:
            yield self
            return
        xs = {self.x1, self.x2}
        ys = {self.y1, self.y2}
        if self.x1 < other.x1 < self.x2:
            xs.add(other.x1)
        if self.x1 < other.x2 < self.x2:
            xs.add(other.x2)
        if self.y1 < other.y1 < self.y2:
            ys.add(other.y1)
        if self.y1 < other.y2 < self.y2:
            ys.add(other.y2)
        for (x1, x2), (y1, y2) in itertools.product(
            pairwise(sorted(xs)), pairwise(sorted(ys))
        ):
            rect = type(self)(x1, y1, x2, y2)
            if rect != inter:
                yield rect
    __sub__ = difference

    def __init__(self, x1, y1, x2, y2):
        if x1 > x2 or y1 > y2:
            raise ValueError("Coordinates are invalid")
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2

    def __iter__(self):
        yield self.x1
        yield self.y1
        yield self.x2
        yield self.y2

    def __eq__(self, other):
        return isinstance(other, Rectangle) and tuple(self) == tuple(other)

    def __ne__(self, other):
        return not (self == other)

    def __repr__(self):
        return type(self).__name__+repr(tuple(self))


def pairwise(iterable):
    # https://docs.python.org/dev/library/itertools.html#recipes
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)

def NormalizedRectangle(Rectangle):
    resolution = (800, 450)
    def __init__(self, center, width, height):
        print('h')
        x1 = np.clip(center[0] - width, 0, 1)
        x2 = np.clip(center[0] + width, 0, 1)
        y1 = np.clip(center[1] - height, 0, 1)
        y2 = np.clip(center[1] + height, 0, 1)
        # x1 = np.clip(int((center[0] - width) * self.resolution[0]), 0, self.resolution[0])
        # x2 = np.clip(int((center[0] + width) * self.resolution[0]), 0, self.resolution[0])
        # y1 = np.clip(int((center[1] - height) * self.resolution[1]), 0, self.resolution[1])
        # y2 = np.clip(int((center[1] + height) * self.resolution[1]), 0, self.resolution[1])
        self.width = width
        self.height = height
        self.center = np.array(center)
        self.area = width * height
        super(NormalizedRectangle, self).__init__(x1, y1, x2, y2)

    def get_center(self, res = (800, 450)):
        return np.int0(np.dot(self.center, res))

    def get_shape(self, res = (800, 450)):
        return np.int0([self.width * res[0], self.height * res[1]])

    def get_rect(self, res = (800, 450)):
        # res: (height, width)
        return np.int0([self.x1 * res[0], self.y1 * res[1], self.width * res[0], self.height * res[1]])

    def get_bound(self, res = (800, 450)):
        return int(self.x1 * res[0]), int(self.x2 * res[1]), int(self.y1 * res[1]), int(self.y2 * res[1])

    def inside(self, pt, res = (800, 450)):
        if pt[0] <= self.x2 and pt[0] >= self.x1 and pt[1] <= self.y2 and pt[1] >= self.y1:
            return True
        else:
            return False

    def inside_cv(self, pt, res = (800, 450)):
        x1, x2, y1, y2 = self.get_bound()
        if pt[0] <= res[0] - x1 and pt[0] >= res[1] - x2 and pt[1] <= y2 and pt[1] >= y1:
            return True
        else:
            return False

    def get_real_bound(self, res):
        xmin, xmax, ymin, ymax = (int)(self.xmin * res[0]), (int)(self.xmax * res[0]), (int)(self.ymin * res[1]), (int)(self.ymax * sres[1])
        return xmin, xmax, ymin, ymax

# class Rect:
#     def __init__(self, center, radius, width, height):
#         self.center = center
#         self.radius = radius
#         self.xmin = np.clip (center[0] - radius, 0, width)
#         self.xmax = np.clip (center[0] + radius, 0, width)
#         self.ymin = np.clip (center[1] - radius, 0, height)
#         self.ymax = np.clip (center[1] + radius, 0, height)

#     def get_bound(self):
#         return (int)self.xmin, (int)self.xmax, (int)self.ymin, (int)self.ymax


class NormalizedRect:
    def __init__(self, center, width, height, res = (800, 450)):
        """ A normalized rect
        """
        self.res = res
        self.center = np.array(center)
        self.width = width
        self.height = height
        self.resolution = res
        self.aspect_ratio = self.resolution[0] / self.resolution[1]
        self.xmin = np.clip(center[0] - width, 0, 1)
        self.xmax = np.clip(center[0] + width, 0, 1)
        self.ymin = np.clip(center[1] - height, 0, 1)
        self.ymax = np.clip(center[1] + height, 0, 1)
        self.area = (self.xmax - self.xmin) * (self.ymax - self.ymin)
        self.shape = ((int)(self.xmax - self.xmin),
                      (int)(self.ymax - self.ymin))
        self.corners = np.asarray(
            [(self.xmin, self.ymin), (self.xmin, self.ymax), (self.xmax, self.ymax), (self.xmax, self.ymin)])

    def inside(self, pt):
        if pt[0] > 1:
            pt = (pt[0] / self.res[0], pt[1] / self.res[1])
        if pt[0] <= self.xmax and pt[0] >= self.xmin and pt[1] <= self.ymax and pt[1] >= self.ymin:
            return True
        else:
            return False

    def intersect(self, rect):
        if self.xmin < rect.xmax and self.xmax > rect.xmin and self.ymax > rect.ymin and self.ymin < rect.ymax:
            xmin = max(self.xmin, rect.xmin)
            xmax = min(self.xmax, rect.xmax)
            ymin = max(self.ymin, rect.ymin)
            ymax = min(self.ymax, rect.ymax)
            center = ((xmin + xmax) / 2.0,  (ymin + ymax) / 2.0)
            rect = NormalizedRect(center, xmax - xmin,
                                  ymax - ymin, (self.height, self.width))
            return rect
        else:
            return None

    def get_bound(self):
        return self.xmin, self.xmax, self.ymin, self.ymax

    def get_real_bound(self):
        return (int)(self.xmin * self.res[0]), (int)(self.xmax * self.res[0]), (int)(self.ymin * self.res[1]), (int)(self.ymax * self.res[1])

    def __repr__(self):
        return repr(self.xmin) + repr(self.xmax) + repr(self.ymin) + repr(self.ymax)

SelectionType = {'REGION_SELECTION': 0, 'OBJECT_SELECTION': 1}


class Selection(TrackerBase):
    TYPE = {'OBJECT_SELECTION': 0, 'AREA_SELECTION': 1}
    resolution = (800, 450)

    def __init__(self, msg):
        super(Selection, self).__init__()
        self.id = msg.guid
        self.type = msg.type
        debug_log('msg: ', msg.center, msg.width, msg.height)
        self.normalized_rect = NormalizedRect(
            (msg.center.x, msg.center.y), msg.width, msg.height)
        if self.type == self.TYPE['OBJECT_SELECTION']:
            self.detected = False

    # @report_type_error('Input variable should be a RegionSelection msg.')
    def update(self, msg):
        self.type = msg.type
        self.alpha = 0.2
        self.normalized_rect = NormalizedRect(
            (msg.center.x, msg.center.y), msg.width, msg.height)

    def get_mask(self):
        mask = np.zeros(self.resolution, dtype='uint8')
        xmin, xmax, ymin, ymax = self.normalized_rect.get_real_bound(self.resolution)
        mask[xmin:xmax, ymin:ymax] = True
        return mask

    def draw(self, img):
        x1, x2, y1, y2 = self.normalized_rect.get_real_bound()
        debug_log(x1, x2, y1, y2)
        debug_log(self.normalized_rect.xmin, self.normalized_rect.xmax, self.normalized_rect.ymin, self.normalized_rect.ymax)
        debug_img = img.copy()
        cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255,0,0), 2)
        return debug_img


class SelectionManager(TrackingManagerBase):
    def __init__(self, res):
        #: list of Selection:
        self.selections = {}
        self.resolution = res
        Selection.resolution = res
        # self.region_selection_server = rospy.Service(
        #     'region_selection', RegionSelection, self.region_selection_callback)
        self.depth_image = None

    def update_image(self, depth_image, rgb_image):
        self.depth_image = depth_image
        self.rgb_image = rgb_image

    # @report_type_error('Input variable should be a list of msgs.')
    def update(self, selection_msgs):
        for msg in selection_msgs:
            selection = self.selections.get(msg.guid)
            debug_log(selection)
            if selection is not None:
                selection.update(msg)
            else:
                self.selections[msg.guid] = Selection(msg)

    def get_selections_with_type(self, selection_type):
        list_of_selections = list(self.selections.values())
        return [s for s in list_of_selections if s.type == selection_type]

    def get_selections(self):
        return list(self.selections.values())
    
    def get_rects(self):
        return [s.normalized_rect for s in self.selections.values()]

    def draw(self, img):
        debug_img = img.copy()
        if len(self.selections.values()) != 0:
            debug_log('Drawing selection')
            for s in self.selections.values():
                debug_img = s.draw(debug_img)
        return debug_img

    # def region_selection_callback(self, req):
    #     self.update([req])
    #     region = self.selections.get(req.guid)
    #     # TODO: detect objects
    #     response = RegionSelectionResponse()
    #     response.success = True
    #     response.msg = 'Obejcts found.'
    #     return response

    # @staticmethod
    # def detect_objects(depth_img, rgb_img, selection):
    #     mask = self.selection.get_mask()
    #     masked_depth = cv2.bitwise_and(depth_img, depth_img, mask = mask)
    #     masked_rgb = cv2.bitwise_and(rgb_img, rgb_img, mask = mask)
    #     masked_depth[masked_depth < 5] = 0