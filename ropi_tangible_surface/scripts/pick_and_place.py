#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import rospy
import actionlib
import tf
from tf.transformations import *
from copy import deepcopy

import cv2

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import WrenchStamped, Vector3

from icl_phri_robotiq_control.robotiq_utils import RobotiqActionClient
from inverseKinematicsUR5 import InverseKinematicsUR5, transformRobotParameter


class PickNPlace(object):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    INIT_POS = np.asarray([-0.00988322893251592, -1.1638777891742151, 1.1179766654968262, -
                           1.4695408979998987, -1.600640122090475, -0.0019872824298303726])  # , 0.2823529411764706]
    SPEED = 3.0

    def __init__(self):
        self.gripper_offset = 0.15
        self.pick_offset = 0.15
        self.listener = tf.TransformListener()
        self.transformer = tf.TransformerROS()
        self.cancel = False
        self.init_gripper('icl_phri_gripper/gripper_controller')
        self.init_ur5('icl_phri_ur5/follow_joint_trajectory')
        pts2 = np.array([[[-0.633, 0.565], [-0.702, 0.433], [-0.662, 0.284], [-0.456, 0.266], [-0.427, 0.399], [-0.410,
                                                                                                                0.588], [-0.135, 0.592], [-0.159, 0.421], [-0.115, 0.256], [0.156, 0.230], [0.172, 0.430], [0.195, 0.626]]])
        pts1 = np.array([[83, 70], [81, 186], [80, 309], [251, 330], [261, 217], [268, 77], [
                        485, 81], [480, 210], [495, 337], [691, 349], [722, 199], [733, 49]])
        self.M, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)

    def coord_converter(self, pt):
        solution = np.matmul(self.M, np.array([pt[0], pt[1], 1]))
        solution = solution / solution[2]
        return solution[0], solution[1]

    def init_gripper(self, topic):
        self.gripper_ac = RobotiqActionClient(topic)
        self.gripper_ac.wait_for_server()
        self.gripper_ac.initiate()
        self.gripper_ac.send_goal(0.10)
        self.gripper_ac.wait_for_result()

    def close_gripper(self):
        self.gripper_ac.send_goal(0.0)
        self.gripper_ac.wait_for_result()

    def open_gripper(self):
        self.gripper_ac.send_goal(0.1)
        self.gripper_ac.wait_for_result()

    def init_ur5(self, topic):
        self.client = actionlib.SimpleActionClient(
            topic, FollowJointTrajectoryAction)
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = self.JOINT_NAMES
        print ("Waiting for server...")
        self.client.wait_for_server()
        print ("Connected to server")
        joint_states = rospy.wait_for_message("joint_states", JointState)
        print(joint_states)
        joint_states = list(deepcopy(joint_states).position)
        del joint_states[-1]
        self.joints_pos_start = np.array(joint_states)
        self.move_joints(self.INIT_POS, self.SPEED * 2)
        joint_weights = [12, 5, 4, 3, 2, 1]
        self.ik = InverseKinematicsUR5()
        self.ik.setJointWeights(joint_weights)
        self.ik.setJointLimits(-math.pi, math.pi)
        # self.sub = rospy.Subscriber('/target_position', Int32MultiArray, self.pickplace_cb)
        print ("Init done")

    def move_joints(self, joints, duration=3.0):
        if joints is not None and not self.cancel:
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[
                    0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=joints.tolist(), velocities=[
                    0]*6, time_from_start=rospy.Duration(duration)),
            ]
            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = joints
                self.client.wait_for_result()
            except KeyboardInterrupt:
                self.client.cancel_goal()
                raise
            except:
                raise
        elif joints is None:
            rospy.loginfo("fail to find IK solution")
        elif self.cancel:
            rospy.logwarn("this goal canceled")

    def move_traj(self, traj, duration=3.0):
        if len(traj) != 0 and not self.cancel:
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[
                    0]*6, time_from_start=rospy.Duration(0.0))]
            for i, joints in enumerate(traj):
                self.goal.trajectory.points.append(JointTrajectoryPoint(positions=joints.tolist(), velocities=[
                    0]*6, time_from_start=rospy.Duration((i + 1) * duration)))
            # self.goal.trajectory.points.append(JointTrajectoryPoint(positions=self.INIT_POS.tolist(), velocities=[
            #         0]*6, time_from_start=rospy.Duration((len(traj) + 1) * duration)))
            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = traj[-1]
                self.client.wait_for_result()
            except KeyboardInterrupt:
                self.client.cancel_goal()
                raise
            except:
                raise
        elif traj is None:
            rospy.loginfo("fail to find IK solution")
        elif self.cancel:
            rospy.logwarn("this goal canceled")

    def define_grasp(self, position):
        quat = tf.transformations.quaternion_from_euler(3.14, 0, -3.14)
        dest_m = self.transformer.fromTranslationRotation(position, quat)
        return dest_m

    def move(self, position):
        qsol = self.ik.findClosestIK(position, self.joints_pos_start)
        self.move_joints(qsol)

    def find_ik(self, traj):
        print(self.joints_pos_start.shape)
        prev_pos = self.joints_pos_start
        output = []
        for coord in traj:
            # print(coord, prev_pos)
            prev_pos = self.ik.findClosestIK(coord, prev_pos)
            output.append(deepcopy(prev_pos))
        return output

    def pick_and_place(self, pt1, pt2, object_height):
        traj = self.generate_trajectory(pt1, pt2, object_height)
        self.move_traj(traj[:2])
        print("grasp")
        self.close_gripper()
        self.move_traj(traj[2:])
        print("release")
        self.open_gripper()
        print('home')
        self.move_joints(self.INIT_POS)

    def generate_trajectory(self, pick_pt, place_pt, object_height):
        pick_coord = self.coord_converter(pick_pt)
        place_coord = self.coord_converter(place_pt)
        pick_coord = np.array(
            [pick_coord[0], pick_coord[1], self.gripper_offset + object_height * 0.3])
        place_coord = np.array(
            [place_coord[0], place_coord[1], self.gripper_offset + object_height * 0.3])
        traj = [pick_coord + [0, 0, object_height + self.pick_offset], pick_coord, pick_coord + [0, 0, self.pick_offset],
                place_coord + [0, 0, self.pick_offset], place_coord]
        traj_defined = map(lambda x: self.define_grasp(x), traj)
        # print(traj[0].shape)
        # print(list(traj_defined)[0].shape)
        joint_traj = self.find_ik(traj_defined)
        return joint_traj


if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    task = pick_and_place()
    task.test()
    rospy.spin()
