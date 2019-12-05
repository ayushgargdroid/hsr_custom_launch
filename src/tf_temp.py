#!/usr/bin/env python
import rospy
import tf
import actionlib
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import String

rospy.init_node('listener', anonymous=True)
br = tf.TransformBroadcaster()
tf_transformer = tf.Transformer()
tf_listen = tf.TransformListener()
tf_ros = tf.TransformerROS()
rate = rospy.Rate(10)
trans = rot = None
while rospy.is_shutdown() or trans is None:
    try:
        (trans,rot) = tf_listen.lookupTransform('/head_rgbd_sensor_link', '/object',rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rate.sleep()
        continue
    rate.sleep()

trans[2] = 0.962

while not rospy.is_shutdown():
    br.sendTransform(trans,
            rot,
            rospy.Time.now(),
            "object2",
            "head_rgbd_sensor_link")
    rate.sleep()

