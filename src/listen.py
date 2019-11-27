#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", str(data.height),str(data.width))
    rospy.loginfo('T shape '+data.data)
    np.save('t.npy',data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
