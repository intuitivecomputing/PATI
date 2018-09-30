#!/usr/bin/env python
from ropi_tangible_surface.common_imports import *
import rospy
import actionlib
import tf
from tf.transformations import *
from copy import deepcopy
import inspect

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
# ee:/tool0 base:/base

from ropi_msgs.srv import GripperControl


def rad2deg(r): return (r / math.pi * 180.)


def deg2rad(d): return (d / 180. * math.pi)


class GripperServiceClient(object):
    def __init__(self, topic):
        rospy.wait_for_service(topic)
        self.srv_proxy = rospy.ServiceProxy(topic, GripperControl)

    def move_to(self, pos):
        # print('moveto: ', pos)
        pos = 255 - int(pos / 0.14 * 255)
        # print('moveto: ', pos)
        try:
            resp1 = self.srv_proxy(pos)
            return resp1.success
        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)

    def close(self):
        try:
            resp1 = self.srv_proxy(255)
            return resp1.success
        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)

    def open(self):
        try:
            resp1 = self.srv_proxy(0)
            return resp1.success
        except rospy.ServiceException, e:
            print ("Service call failed: %s" % e)


        
UR_STATES = {'PENDING': 0,
            'ACTIVE': 1,
            'PREEMPTED': 2,
            'SUCCEEDED': 3,
            'ABORTED': 4,
            'REJECTED': 5,
            'PREEMPTING': 6,
            'RECALLING': 7,
            'RECALLED': 8,
            'LOST': 9}

get_ur5_status = lambda s: [k for k, v in UR_STATES.items() if v == s][0]

class UR5MissionError(Exception):
    def __init__(self, err_code, function_name):
        super(UR5MissionError, self).__init__(err_code, function_name)
        self.err_code = err_code
        self.err_state = get_ur5_status(err_code)
        self.message = '[{}]: UR5 mission {}'.format(function_name, self.err_state)

class PickNPlace(object):
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    INIT_JOINTS = np.asarray([-0.00988322893251592, -1.1638777891742151, 1.1179766654968262, -
                              1.4695408979998987, -1.600640122090475, -0.0019872824298303726])  # , 0.2823529411764706]
    SPEED = 3.0

    def __init__(self):
        self.gripper_offset = 0.15
        self.pick_offset = 0.15
        self.listener = tf.TransformListener()
        self.transformer = tf.TransformerROS()
        self.cancel = False
        self.init_gripper_service('/gripper_control')
        # self.init_gripper('icl_phri_gripper/gripper_controller')
        self.init_ur5('icl_phri_ur5/follow_joint_trajectory')
        # pts2 = np.array([[[-0.633, 0.565], [-0.702, 0.433], [-0.662, 0.284], [-0.456, 0.266], [-0.427, 0.399], [-0.410,
        #                                                                                                         0.588], [-0.135, 0.592], [-0.159, 0.421], [-0.115, 0.256], [0.156, 0.230], [0.172, 0.430], [0.195, 0.626]]])
        # pts1 = np.array([[83, 70], [81, 186], [80, 309], [251, 330], [261, 217], [268, 77], [
        #                 485, 81], [480, 210], [495, 337], [691, 349], [722, 199], [733, 49]])
        pts1 = np.array([[76, 374], [78, 284], [81, 165], [73, 47], [239, 41], [244, 137], [254, 262], [273, 389], [414, 390], [410, 279], [414, 152], [414, 35], [570, 28], [574, 152], [583, 280], [589, 386], [753, 397], [748, 290], [750, 158], [761, 33], [119, 324], [127, 235], [127, 125], [129, 40], [265, 44], [270, 131], [276, 251], [281, 339], [402, 338], [410, 251], [421, 136], [424, 37], [557, 42], [549, 158], [556, 265], [549, 352], [689, 355], [695, 260]])
        pts2 = np.array([[[-0.612, 0.198], [-0.620, 0.320], [-0.634, 0.479], [-0.635, 0.640], [-0.421, 0.647], [-0.412, 0.511], [-0.399, 0.334], [-0.344, 0.176], [-0.166, 0.171], [-0.171, 0.323], [-0.166, 0.495], [-0.168, 0.666], [0.049, 0.667], [0.051, 0.499], [0.058, 0.324],  [0.062, 0.184], [0.291, 0.175], [0.291, 0.312], [0.288, 0.482], [0.310, 0.653], [-0.550, 0.269], [-0.550, 0.399], [-0.548, 0.543], [-0.544, 0.660], [-0.365, 0.660], [-0.374, 0.533], [-0.363, 0.386], [-0.349, 0.259], [-0.192, 0.250], [-0.179, 0.368], [-0.164, 0.515], [-0.151, 0.671], [0.034, 0.668], [0.009, 0.500], [0.014, 0.361], [0.020, 0.247], [0.204, 0.233], [0.218, 0.354]]])
        self.M, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 5.0)
        print('j', self.get_joints())

    def lookup_pos(self):
        try:
            (trans, rot) = self.listener.lookupTransform(
                '/base', '/tool0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Lookup Transform failed!!!')
            return [0, 0, 0], None
        return trans, rot

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

    def init_gripper_service(self, topic):
        rospy.loginfo('Waiting for gripper service.')
        self.gripper_sc = GripperServiceClient(topic)
        self.open_gripper()

    def close_gripper(self):
        # self.gripper_ac.send_goal(0.0)
        # self.gripper_ac.wait_for_result()
        self.gripper_sc.close()

    def open_gripper(self):
        # self.gripper_ac.send_goal(0.1)
        # self.gripper_ac.wait_for_result()
        self.gripper_sc.open()

    def send_script(self, script):
        rospy.loginfo('sending script: {}'.format(script))
        self.script_pub.publish(String(script))

    def init_ur5(self, topic):
        self.script_pub = rospy.Publisher(
            "/icl_phri_ur5/ur_driver/URScript", String, queue_size=1)
        self.client = actionlib.SimpleActionClient(
            topic, FollowJointTrajectoryAction)
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = self.JOINT_NAMES
        print("Waiting for server...")
        self.client.wait_for_server()
        print("Connected to server")
        joint_states = rospy.wait_for_message("joint_states", JointState)
        print(joint_states)
        joint_states = list(deepcopy(joint_states).position)
        del joint_states[-1]
        self.joints_pos_start = np.array(joint_states)
        joint_weights = [12, 5, 4, 3, 2, 1]
        self.ik = InverseKinematicsUR5()
        self.ik.setJointWeights(joint_weights)
        self.ik.setJointLimits(-math.pi, math.pi)
        trans, rot = self.lookup_pos()
        initial_pos = [-0.651, -0.100, 0.410]
        # self.send_script('movel([0, 0, 0.3, 0, 0, 0], a=0.1, v=0.1)')
        if np.linalg.norm(np.asarray(initial_pos) - trans) > 0.02:
            try:
                trans[2] = initial_pos[2]
                self.move(self.define_grasp(trans))
                self.move_joints(self.INIT_JOINTS, self.SPEED * 2)
            except:
                rospy.logerr('init failed, fall back')
                self.move_joints(self.INIT_JOINTS, self.SPEED * 2)
                raise

        # self.sub = rospy.Subscriber('/target_position', Int32MultiArray, self.pickplace_cb)
        print("Init done")

    @staticmethod
    def get_joints():
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joint_states = list(deepcopy(joint_states).position)
        return np.asarray(joint_states[:-1])
        
    def move_joints(self, joints, duration=3.0):
        self.joints_pos_start = self.get_joints()
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
                rospy.loginfo('[move_joints] Result: {}'.format(self.client.wait_for_result()))
                finish_state = self.client.get_state()
                if (not finish_state == UR_STATES['ACTIVE'] and not finish_state == UR_STATES['SUCCEEDED']):
                    raise UR5MissionError(finish_state, inspect.stack()[0][3])
            except KeyboardInterrupt:
                self.client.cancel_goal()
                raise
            except UR5MissionError as e:
                rospy.logerr(e.message)
                self.client.cancel_all_goals()
                raise
        elif joints is None:
            rospy.loginfo("fail to find IK solution")
        elif self.cancel:
            rospy.logwarn("this goal canceled")

    def move_traj(self, traj, duration=3.0):
        self.joints_pos_start = self.get_joints()
        if len(traj) != 0 and not self.cancel:
            self.goal.trajectory.points = [
                JointTrajectoryPoint(positions=self.joints_pos_start.tolist(), velocities=[
                    0]*6, time_from_start=rospy.Duration(0.0))]
            for i, joints in enumerate(traj):
                self.goal.trajectory.points.append(JointTrajectoryPoint(positions=joints.tolist(), velocities=[
                    0]*6, time_from_start=rospy.Duration((i + 1) * duration)))
            # self.goal.trajectory.points.append(JointTrajectoryPoint(positions=self.INIT_JOINTS.tolist(), velocities=[
            #         0]*6, time_from_start=rospy.Duration((len(traj) + 1) * duration)))
            try:
                self.client.send_goal(self.goal)
                self.joints_pos_start = traj[-1]
                # self.client.wait_for_result()
                rospy.loginfo('[move_traj] Result: {}'.format(self.client.wait_for_result()))
                finish_state = self.client.get_state()
                if (not finish_state == UR_STATES['ACTIVE'] and not finish_state == UR_STATES['SUCCEEDED']):
                    raise UR5MissionError(finish_state, inspect.stack()[0][3])
            except KeyboardInterrupt:
                self.client.cancel_goal()
                raise
            except UR5MissionError as e:
                rospy.logerr(e.message)
                self.client.cancel_all_goals()
                raise
        elif traj is None:
            rospy.loginfo("fail to find IK solution")
        elif self.cancel:
            rospy.logwarn("this goal canceled")

    def define_grasp(self, position, angle=0):
        print('amgle ', angle)
        quat = tf.transformations.quaternion_from_euler(math.pi, 0, deg2rad(angle)) #(math.pi, 0, 0)
        dest_m = self.transformer.fromTranslationRotation(position, quat)
        return dest_m

    def move(self, position):
        qsol = self.ik.findClosestIK(position, self.joints_pos_start)
        self.move_joints(qsol)

    def find_ik(self, traj):
        # print(self.joints_pos_start.shape)
        prev_pos = self.joints_pos_start
        output = []
        for coord in traj:
            # print(coord, prev_pos)
            prev_pos = self.ik.findClosestIK(coord, prev_pos)
            output.append(deepcopy(prev_pos))
        return output

    # pt: [[x,y], angle]
    def pick_and_place(self, pt1, pt2, object_height, object_diameter=0.14):
        try:
            self.gripper_sc.move_to(np.clip(object_diameter + 0.05, 0, 0.14))
            traj = self.generate_trajectory(pt1, pt2, object_height)
            # print('traj: {}'.format(traj))
            self.move_traj(traj[:2])
            rospy.loginfo("grasp")
            self.close_gripper()
            self.move_traj(traj[2:-1])
            rospy.loginfo("release")
            self.gripper_sc.move_to(np.clip(object_diameter + 0.02, 0, 0.14))
            self.move_traj([traj[-1]])
            self.open_gripper()
            # self.move_joints(self.INIT_JOINTS)
        except:
            raise

    def pick_and_place_mission(self, mission):
        # mission: GraspDataClass[]
        print('ppmis: {}'.format(mission))
        for i, m in enumerate(mission):
            rospy.loginfo('Executing mission #{}'.format(i))
            if m.target_position is not None:
                print(m.position, m.target_position, m.height)
                try:
                    self.pick_and_place([m.position, m.angle], [m.target_position, m.angle], m.height, m.diameter)
                except:
                    rospy.logerr('mission #{} failed!!!'.format(i))
                    self.move_joints(self.INIT_JOINTS)
                    raise
            else:
                rospy.logerr('No target set for mission #{}'.format(i))

        self.move_joints(self.INIT_JOINTS)

    # pick/place_pos: [[x,y], angle]
    def generate_trajectory(self, pick_pose, place_pose, object_height):
        pick_coord = self.coord_converter(pick_pose[0][:2])
        place_coord = self.coord_converter(place_pose[0][:2])
        # print(pick_coord)
        pick_coord = np.array(
            [pick_coord[0], pick_coord[1], self.gripper_offset + object_height * 0.3])
        place_coord = np.array(
            [place_coord[0], place_coord[1], self.gripper_offset + object_height * 0.3])
        pos_traj = [pick_coord + [0, 0, object_height + self.pick_offset],
                pick_coord,
                pick_coord + [0, 0, self.pick_offset],
                place_coord + [0, 0, self.pick_offset],
                place_coord,
                place_coord + [0, 0, object_height + self.pick_offset]]
        angle_traj = [pick_pose[1]] * 3 +[place_pose[1]] * 3
        traj = zip(pos_traj, angle_traj)
        traj_defined = map(lambda x: self.define_grasp(x[0], x[1]), traj)
        # print(traj)
        # print(list(traj_defined)[0].shape)
        joint_traj = self.find_ik(traj_defined)
        return joint_traj


if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    task = PickNPlace()
    # task.pick_and_place((412, 211), (89, 196), 0.15)
    pos = task.define_grasp([-0.651, -0.103, 0.410], angle = 20)
    print('defgrasp: ', pos)
    task.move(pos)
    rospy.spin()
