#!/usr/bin/env python
import rospy
import tf
import actionlib
import numpy as np
from gqcnn.srv import GQCNNGraspPlannerRequest, GQCNNGraspPlannerResponse, GQCNNGraspPlanner

import trajectory_msgs.msg
import move_base_msgs.msg 
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()
color_image = depth_image = camera_info = None

def camera_info_callback(data):
    global camera_info
    camera_info = data

def color_callback(data):
    global bridge, color_image
    # color_image = bridge.imgmsg_to_cv2(data,'passthrough')
    color_image = data
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", str(data.height),str(data.width))
    # rospy.loginfo('T shape '+data.data)
    # depth_image = bridge.imgmsg_to_cv2(data, 'passthrough')
    # rospy.loginfo('size: '+str(depth_image.shape))
    # np.save('t.npy',depth_image)

def depth_callback(data):
    global bridge, depth_image
    t = bridge.imgmsg_to_cv2(data, 'passthrough')
    t = t.astype(np.float32)/1000.0
    depth_image = bridge.cv2_to_imgmsg(t,'passthrough')
    # depth_image = data
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", str(data.height),str(data.width))
    # rospy.loginfo('T shape '+data.data)
    # rospy.loginfo('size: '+str(depth_image.shape))
    # np.save('t.npy',depth_image)
    
def listener():
    global depth_image, color_image, camera_info
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.loginfo('Attaching to Move Base Action Server')
    client = actionlib.SimpleActionClient('move_base/move', move_base_msgs.msg.MoveBaseAction)
    pub = rospy.Publisher('/hsrb/head_trajectory_controller/command',trajectory_msgs.msg.JointTrajectory, queue_size=10)
    client.wait_for_server()
    move_goal = move_base_msgs.msg.MoveBaseGoal()
    # move_goal.header.seq = move_goal.goal.target_pose.header.seq = 1
    # move_goal.header.stamp = rospy.Time.now()
    # move_goal.goal.target_pose.header.stamp = rospy.Time.now()
    move_goal.target_pose.header.frame_id = 'map'
    move_goal.target_pose.pose.orientation.z = -0.7071
    move_goal.target_pose.pose.orientation.w = 0.7071
    client.send_goal(move_goal)
    rospy.loginfo('Sent command to Move Base to turn')
    client.wait_for_result()

    rospy.loginfo("Now tilting and panning the head")
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [3.14, -1.14]
    p.velocities = [0, 0]
    p.time_from_start = rospy.Time(3)
    traj.points = [p]

    # publish ROS message
    pub.publish(traj)

    rospy.sleep(5.0)

    rospy.loginfo('Now subscribing to image topics')
    rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, depth_callback)
    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, color_callback)

    br = tf.TransformBroadcaster()
    tf_transformer = tf.Transformer()
    tf_listen = tf.TransformListener()
    tf_ros = tf.TransformerROS()
    rate = rospy.Rate(10)
    grasp_pose = None
    xt = None
    rospy.loginfo('Now waiting for service')
    while not rospy.is_shutdown():
        if(depth_image is None):
            rospy.loginfo('depth not here')
        if(color_image is None):
            rospy.loginfo('color not here')
        if(camera_info is None):
            rospy.loginfo('info not here')
        if(depth_image is not None and color_image is not None and camera_info is not None):
            rospy.loginfo('Got service')
            grasp_service = rospy.ServiceProxy('/grasp_planner',GQCNNGraspPlanner)
            xt = bridge.imgmsg_to_cv2(depth_image)
            srv = GQCNNGraspPlannerRequest()
            srv.color_image = color_image
            srv.depth_image = depth_image
            srv.camera_info = camera_info
            resp = grasp_service(srv)
            rospy.loginfo(resp.grasp.pose)
            grasp_pose = resp.grasp.pose
            rospy.loginfo(resp.grasp.angle)
            rospy.loginfo(resp.grasp.depth)
            rospy.loginfo(resp.grasp.center_px)
            rospy.loginfo(xt[int(resp.grasp.center_px[1]),int(resp.grasp.center_px[0])])
            break

        rate.sleep()

    trans = rot = None
    while rospy.is_shutdown() or trans is None:
        try:
            (trans,rot) = tf_listen.lookupTransform('/odom', '/head_rgbd_sensor_link',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()

    t = tf.transformations.euler_from_quaternion([grasp_pose.orientation.x,grasp_pose.orientation.y,grasp_pose.orientation.z,grasp_pose.orientation.w])
    rot_mat = tf.transformations.euler_matrix(t[0],t[1],t[2])
    trans_mat = tf.transformations.translation_matrix([grasp_pose.position.x,grasp_pose.position.y,grasp_pose.position.z])
    transform_mat = np.dot(trans_mat,rot_mat)
    odom_mat = tf_ros.fromTranslationRotation(trans,rot)
    # rot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
    obj_mat = np.dot(odom_mat,transform_mat)
    obj_rot = tf.transformations.quaternion_from_matrix(obj_mat)
    obj_trans = tf.transformations.translation_from_matrix(obj_mat)

    trans_mat = tf.transformations.translation_matrix([grasp_pose.position.x,grasp_pose.position.y,xt[int(resp.grasp.center_px[1]),int(resp.grasp.center_px[0])]])
    transform_mat = np.dot(trans_mat,rot_mat)
    grasp_mat = np.dot(odom_mat,transform_mat)
    grasp_rot = tf.transformations.quaternion_from_matrix(grasp_mat)
    grasp_trans = tf.transformations.translation_from_matrix(grasp_mat)


    change = tf_ros.fromTranslationRotation((0,0,-0.1),(0,0,0,1))
    pregrasp_mat = np.dot(grasp_mat,change)
    pregrasp_rot = tf.transformations.quaternion_from_matrix(pregrasp_mat)
    pregrasp_trans = tf.transformations.translation_from_matrix(pregrasp_mat)

    # grasp_boundary_trans = grasp_trans
    # grasp_boundary_trans[2] = xt[int(resp.grasp.center_px[1]),int(resp.grasp.center_px[0])]

    while not rospy.is_shutdown():
        br.sendTransform(obj_trans,
                obj_rot,
                rospy.Time.now(),
                "object",
                "odom")
        
        br.sendTransform(pregrasp_trans,
                pregrasp_rot,
                rospy.Time.now(),
                "pregrasp",
                "odom")

        br.sendTransform(grasp_trans,
                grasp_rot,
                rospy.Time.now(),
                "grasp",
                "odom")

        rate.sleep()

if __name__ == '__main__':
    listener()