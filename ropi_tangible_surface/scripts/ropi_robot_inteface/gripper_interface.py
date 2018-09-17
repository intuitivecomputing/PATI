#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction

class GripperInterface(object):
    def __init__(self, action_name='icl_phri_gripper/gripper_controller'):
        self.client = actionlib.SimpleActionClient(action_name, GripperCommandAction)