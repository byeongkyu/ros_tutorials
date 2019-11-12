#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
import tf2_ros

from trac_ik_python.trac_ik import IK
import tf2_geometry_msgs
from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.srv import QueryTrajectoryState

class IKTutorial:
    def __init__(self):

        # Read "robot_description" parameter from Parameter Server
        robot_description = rospy.get_param('robot_description')
        pub_cmd = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        rospy.wait_for_service('/arm_controller/query_state')
        rospy.sleep(0.01)

        # Get current state of joints
        try:
            query_state = rospy.ServiceProxy('/arm_controller/query_state', QueryTrajectoryState)
            current_state = query_state(rospy.Time.now())
            print current_state
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

        # Read private parameter of this node
        try:
            base_link = rospy.get_param('~base_link')
            tip_link = rospy.get_param('~tip_link')
            ik_group_name = rospy.get_param('~ik_group_name')
        except KeyError as e:
            rospy.logerr('[%s] Need to set parameter %s'%(rospy.get_name(), e))
            exit(-1)

        # Init IK Solver from Trac IK package
        ik_solver = IK(base_link, tip_link, urdf_string=robot_description)
        current_joint_value = [0.0] * len(ik_solver.joint_names)
        rospy.loginfo(ik_solver.joint_names)

        # Set Target Pose (This point is in workspace)
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"

        target_pose.pose.position.x = 0.8
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.8

        q_target = quaternion_from_euler(0.0, 1.5707, 0.0)
        target_pose.pose.orientation.x = q_target[0]
        target_pose.pose.orientation.y = q_target[1]
        target_pose.pose.orientation.z = q_target[2]
        target_pose.pose.orientation.w = q_target[3]

        # Set Current State to Seed State
        seed_state = current_state.position

        # Find IK Solution
        result = ik_solver.get_ik(seed_state,
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
            target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w)

        # If result is not None, it has solution(s)
        if result != None:
            print result
            cmd = JointTrajectory()
            cmd.header.stamp = rospy.Time.now()
            cmd.joint_names = list(ik_solver.joint_names)

            point = JointTrajectoryPoint()
            for i in result:
                point.positions.append(i)
            point.time_from_start = rospy.Duration(1.0)

            cmd.points.append(point)

            for i in range(5):
                cmd.header.stamp = rospy.Time.now()
                pub_cmd.publish(cmd)
                rospy.sleep(0.1)

        else:
            print "ERROR"

        # Done





if __name__ == '__main__':
    rospy.init_node('ik_tutorial', anonymous=False)
    try:
        m = IKTutorial()
        rospy.spin()
    except rospy.ROSInterruptException: pass