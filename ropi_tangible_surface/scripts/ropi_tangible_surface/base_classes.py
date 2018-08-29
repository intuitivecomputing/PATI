#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import uuid

class TrackerBase(object):
    def __init__(self):
        self.id = uuid.uuid4()

    def update(self, updates):
        raise NotImplementedError



class TrackingManagerBase(object):
    def __init__(self):
        # self.trackers = []
        pass

    def update(self, updates):
        raise NotImplementedError