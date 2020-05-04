#!/usr/bin/env python3

"""
image.py
Author: Samuel Love
Year: 2020

Reads compressed RGB image topic and calculates
mean pixel value with a specified interval.
Calculated value is returned to the console
"""

import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Import ROS messages
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

def callback(data):
   imageSum = 0
   step = 40
   bridge = CvBridge()
   cv_image = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
   # Iterate through image to calcuate mean
   for i in range(0, cv_image.shape[0], step):
       for j in range(0, cv_image.shape[1], step):
           for k in range(3):
           	imageSum += cv_image[i][j][k]
   imageAvg = (imageSum*step*step) / (cv_image.shape[0]*cv_image.shape[1]*cv_image.shape[2])
   print("Average pixel value is: " + str(imageAvg))

def meanImage():
    image = CompressedImage()
    rospy.init_node('imagebrightness', anonymous=True)

    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage,  callback, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    print (sys.version)
    meanImage()

