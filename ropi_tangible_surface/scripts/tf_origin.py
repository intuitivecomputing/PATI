#!/usr/bin/env python  
# py2/3 compatibility
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from future import standard_library
standard_library.install_aliases()
from builtins import *
import sys

import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
# from future.builtins import input
from tf.transformations import *
import os, rospkg
import yaml

if __name__ == '__main__':
    rospy.init_node('tf_recorder')
    set_transform = rospy.get_param('/calibrate_origin', default=False)
    rospack = rospkg.RosPack()
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(150.0)
    
    while not rospy.is_shutdown():
        if set_transform:
            try:
                (trans,rot) = listener.lookupTransform('/kinect2_link', '/aruco_marker_frame', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            print(trans, rot)
            key = input("Press Enter to continue..., f to finish")
            if key == "f":
                set_transform = False
                trans = np.asarray(trans) + [0, 0.02, 0]
                trans = trans + [0.161/2.0, 0.161/2.0, 0]
                q = quaternion_from_euler(-math.pi / 2.0, -math.pi / 2.0, 0)
                rot = quaternion_multiply(rot, q)
                print(trans, rot)
                data = {'ref_frame': '/kinect2_link', 'origin_trans': trans.tolist(), 'origin_rot': rot.tolist()}
                with open(os.path.join(rospack.get_path("ropi_tangible_surface"), "config", "desktop_config.yaml"), 'w') as outfile:
                    yaml.dump(data, outfile)
        else:
            with open(os.path.join(rospack.get_path("ropi_tangible_surface"), "config", "desktop_config.yaml")) as f:
                loaded_data = yaml.load(f)
            br.sendTransform(loaded_data.get('origin_trans'), loaded_data.get('origin_rot'), rospy.Time.now(),"table_origin", loaded_data.get('ref_frame'))
		
        
        rate.sleep()
