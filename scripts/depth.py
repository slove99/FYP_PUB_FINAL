#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math

def callback(data):
   imageSum = 0
   step = 20
   bridge = CvBridge()
   cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
   for i in range(0, data.height, step):
       for j in range(0, data.width, step):
	   if cv_image[i][j] == cv_image[i][j]:
           	imageSum += cv_image[i][j]
   imageAvg = (imageSum*step*step) / (data.height*data.width)
   print("IMAGE AVERAGE IS:  ")
   print(imageAvg)

def depth():
    image = Image()
    rospy.init_node('imagebrightness', anonymous=True)

    rospy.Subscriber('/camera/depth_registered/image_raw', Image,  callback, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    depth()

