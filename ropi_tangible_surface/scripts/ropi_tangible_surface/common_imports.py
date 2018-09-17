#!/usr/bin/env python
# py2/3 compatibility
from __future__ import (absolute_import, division, print_function,
                        unicode_literals)
from future import standard_library
standard_library.install_aliases()
from builtins import *
import sys
import math

import numpy as np


class Angle(object):
    def __init__(self, angle):
        self.angle = angle % 360
        self.radian = self.angle * math.pi / 180

class Radian(object):
    def __init__(self, rad):
        self.radian = rad % (2 * math.pi)
        self.angle = self.radian * 180 / math.pi