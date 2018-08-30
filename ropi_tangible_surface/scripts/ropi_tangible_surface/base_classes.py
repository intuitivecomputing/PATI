#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import uuid


def report_type_error(error_message):
    def decorate(f):
        def applicator(*args, **kwargs):
            try:
                f(*args, **kwargs)
            except (AttributeError, TypeError):
                raise AssertionError(error_message)
        return applicator
    return decorate


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
