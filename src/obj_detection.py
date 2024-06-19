#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
#import cv2

def callback(data):
	rospy.loginfo('recieved an image')

def listener():
	rospy.init_node('obj_detect', anonymous=True)
	rospy.Subscriber("/camera/color/image_raw", Image, callback)
	rospy.spin()

if __name__ =='__main__':
	listener()
