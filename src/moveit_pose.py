#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "whole_body"
    group = moveit_commander.MoveGroupCommander(group_name)
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                 moveit_msgs.msg.DisplayTrajectory,
    #                                                 queue_size=20)
    tf_listen = tf.TransformListener()
    tf_broadcast = tf.TransformBroadcaster()
    tf_ros = tf.TransformerROS()
    trans = rot = None
    rate = rospy.Rate(100)
    while rospy.is_shutdown() or trans is None:
        try:
            (trans,rot) = tf_listen.lookupTransform('/odom', '/object', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
    rospy.loginfo('Position: '+str(trans)+' Orientation: '+str(rot))
    change = tf_ros.fromTranslationRotation((-0.4,0,0),(0,0,0,1))
    rospy.loginfo('\n'+str(change))
    actual = tf_ros.fromTranslationRotation(trans,rot)
    rospy.loginfo('\n'+str(actual))
    pre = np.dot(actual,change)
    rospy.loginfo('\n'+str(pre))
    pre_rot = tf.transformations.quaternion_from_matrix(pre)
    pre_trans = tf.transformations.translation_from_matrix(pre)
    rospy.loginfo('Position: '+str(pre_trans)+' Orientation: '+str(pre_rot))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = pre_rot[3]
    pose_goal.orientation.x = pre_rot[0]
    pose_goal.orientation.y = pre_rot[1]
    pose_goal.orientation.z = pre_rot[2]
    pose_goal.position.x = pre_trans[0]
    pose_goal.position.y = pre_trans[1]
    pose_goal.position.z = pre_trans[2]
    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    while not rospy.is_shutdown():
        tf_broadcast.sendTransform(pre_trans,
                pre_rot,
                rospy.Time.now(),
                "pre",
                "odom")
        rate.sleep()
