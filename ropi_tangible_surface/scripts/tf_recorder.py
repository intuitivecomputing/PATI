#!/usr/bin/env python  
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
from future.builtins import input
import scipy.linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

X,Y = np.meshgrid(np.arange(-3.0, 3.0, 0.5), np.arange(-3.0, 3.0, 0.5))
XX = X.flatten()
YY = Y.flatten()
def fit(data):
    # best-fit linear plane
    A = np.c_[data[:,0], data[:,1], np.ones(data.shape[0])]
    C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])    # coefficients
    print "coeff"
    print C
    # evaluate it on grid
    Z = C[0]*X + C[1]*Y + C[2]
    return Z

def plot(data, Z):
    # plot points and fitted surface
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
    ax.scatter([0], [0], [0], c='b', s=50)
    ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis('equal')
    ax.axis('tight')
    plt.show()
if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    data = []
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/kinect2_link', '/aruco_marker_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        data.append(trans)
        print trans
        key = input("Press Enter to continue..., f to finish")
        if key == "f":
            data = np.array(data)
            Z = fit(data)
            plot(data, Z)
            break
        #rate.sleep()

#  8.86482221e-04 -5.88286502e-01  9.35600193e-01
# [-0.00503571 -0.58703688  0.93483743]
