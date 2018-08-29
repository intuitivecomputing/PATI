#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import uuid

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
    def __init__(self, center, hradius, vradius, (height, width)):
        """ A normalized rect
        """
        self.center = np.array(center)
        self.hradius = hradius
        self.vradius = vradius
        self.aspect_ratio = height / width
        self.width = width
        self.height = height
        self.xmin = np.clip (center[0] - hradius, 0, 1)
        self.xmax = np.clip (center[0] + hradius, 0, 1)
        self.ymin = np.clip (center[1] - vradius, 0, 1)
        self.ymax = np.clip (center[1] + vradius, 0, 1)
        self.area = (self.xmax -self.xmin) * (self.ymax - self.ymin)
        self.shape = ((int)(self.xmax -self.xmin), (int)(self.ymax - self.ymin))
        self.corners = np.asarray([(self.xmin, self.ymin), (self.xmin, self.ymax), (self.xmax, self.ymax), (self.xmax, self.ymin)])

    def inside(self, pt):
        if pt[0] <= self.xmax and pt[0] >= self.xmin and pt[1] <= self.ymax and pt[1] >=self.ymin:
            return True
        else:
            return False
    
    def intersect(self, rect):
        if self.xmin < rect.xmax and self.xmax > rect.xmin and self.ymax > rect.ymin and self.ymin < rect.ymax:
            xmin = max(self.xmin, rect.xmin)
            xmax = min(self.xmax, rect.xmax)
            ymin = max(self.ymin, rect.ymin)
            ymax = min(self.ymax, rect.ymax)
            center = (( xmin + xmax ) / 2.0,  ( ymin + ymax ) / 2.0)
            rect = NormalizedRect(center, xmax - xmin, ymax - ymin, (self.height, self.width))
            return rect
        else:
            return None


    def get_bound(self):
        return self.xmin, self.xmax, self.ymin, self.ymax

    def get_real_bound(self):
        return (int)(self.xmin * self.width), (int)(self.xmax * self.width), (int)(self.ymin * self.height), (int)(self.ymax * self.height)


SelectionType = {'REGION_SELECTION': 0, 'OBJECT_SELECTION': 1}

class Selection(TrackerBase):
    TYPE = {TYPE_OBJECT_SELECTION: 0, TYPE_AREA_SELECTION: 1}
    def __init__(self, msg):
        super(ObjectTracker, self).__init__()
        self.id = msg.guid
        self.type = msg.type
        self.normalized_rect = NormalizedRect(msg.center, msg.hradius, msg.vradius, self.resolution)

    def update(self, selection):
        self.type = selection.type
        self.region = selection.region


class DetectedObject:
    def __init__(self):
        self.uuid = uuid.uuid4()


class SelectionManager:
    def __init__(self):
        #: list of Selection:
        self.selections = []

    def add(self, selection):
        compare_list = [s.guid for s in self.selections]
        if True in compare_list:
            self.selections[compare_list.index(True)].update(selection)
        else:
            self.selections.append(selection)

    def get_selections_with_type(self, selection_type):
        return [s for s in self.selections if s.type == selection_type]