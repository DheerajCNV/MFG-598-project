#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image


def pose_callback(message):
    rospy.loginfo(message)

if __name__ == '__main__':
    rospy.init_node("Image_raw_subscriber")
    sub = rospy.Subscriber('/depth_camera/color/image_raw', Image, callback = pose_callback)

    rospy.loginfo('Node has been started')
    # rospy.loginfo('')
    rospy.spin()
