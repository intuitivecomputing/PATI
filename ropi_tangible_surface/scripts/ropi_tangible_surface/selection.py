#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
from ropi_tangible_surface.base_classes import *
from ropi_msgs.srv import RegionSelection
import uuid
import itertools

import rospy


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
    resolution = ()
    def __init__(self, center, width, height):
        x1 = np.clip(center[0] - width, 0, 1)
        x2 = np.clip(center[0] + width, 0, 1)
        y1 = np.clip(center[1] - height, 0, 1)
        y2 = np.clip(center[1] + height, 0, 1)
        self.width = width
        self.height = height
        self.center = np.array(center)
        self.area = width * height
        super(NormalizedRectangle, self).__init__(x1, y1, x2, y2)

    def get_center(self, res):
        return np.int0(np.dot(self.center, res))

    def get_shape(self, res=(1, 1)):
        return np.int0([self.width * res[1], self.height * res[0]])

    def get_rect(self, res=(1, 1)):
        # res: (height, width)
        return np.int0([self.x1 * res[1], self.y1 * res[0], self.width * res[1], self.height * res[0]])

    def inside(self, pt):
        if pt[0] <= self.x2 and pt[0] >= self.x1 and pt[1] <= self.y2 and pt[1] >= self.y1:
            return True
        else:
            return False
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
    def __init__(self, center, width, height, res):
        """ A normalized rect
        """
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
        return (int)(self.xmin * self.width), (int)(self.xmax * self.width), (int)(self.ymin * self.height), (int)(self.ymax * self.height)


SelectionType = {'REGION_SELECTION': 0, 'OBJECT_SELECTION': 1}


class Selection(TrackerBase):
    TYPE = {'OBJECT_SELECTION': 0, 'AREA_SELECTION': 1}
    resolution = (450, 800)

    def __init__(self, msg):
        super(Selection, self).__init__()
        self.id = msg.guid
        self.type = msg.type
        self.normalized_rect = NormalizedRectangle(
            msg.center, msg.width, msg.height)
        if self.type == TYPE['OBJECT_SELECTION']:
            self.detected = False

    @report_type_error('Input variable should be a RegionSelection msg.')
    def update(self, msg):
        self.type = selection_msg.type
        self.normalized_rect = NormalizedRect(
            msg.center, msg.width, msg.height, self.resolution)


class SelectionManager:
    def __init__(self, res):
        #: list of Selection:
        self.selections = {}
        self.resolution = res
        Selection.resolution = res
        self.region_selection_server = rospy.Service(
            'region_selection', RegionSelection, self.region_selection_callback)
        self.depth_image = None

    def update_image(self, depth_image):
        self.depth_image = depth_image

    @report_type_error('Input variable should be a list of msgs.')
    def update(self, selection_msgs):
        for msg in selection_msgs:
            selection = self.selections.get(msg.guid)
            if selection is not None:
                selection.update(msg)
            else:
                self.selections[msg.guid] = Selection(msg)

    def get_selections_with_type(self, selection_type):
        list_of_selections = list(self.selections.values())
        return [s for s in list_of_selections if s.type == selection_type]

    def get_selections(self):
        return list(self.selections.values())

    def region_selection_callback(self, req):
        self.update([req])
        region = self.selections.get(req.guid)

        response = RegionSelectionResponse()
        response.success = True
        response.msg = 'Obejcts found.'
        return response

    @staticmethod
    def detect_objects(depth_img, selection):
