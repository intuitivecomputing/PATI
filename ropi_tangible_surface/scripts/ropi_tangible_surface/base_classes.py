#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import uuid

class TrackerBase(object):
    def __init__(self):
        self.id = uuid.uuid4()



class TrackingManagerBase(object):
    def __init__(self):
        self.trackers = []