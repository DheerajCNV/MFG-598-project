#!/usr/bin/python2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

def image_callback(msg):

    try:

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        print("Received an image!")
        print(img)
        cv2.imwrite('saved_image.jpg', img)
        cv2.imshow('saved_image.jpg', img)

    except CvBridgeError as e:
        print(e)
    
        


if __name__ == '__main__':

    rospy.init_node('image_converter')
    image_topic = "/depth_camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()