#!/usr/bin/env python

"""
imagestorer.py
Author: Samuel Love
Year: 2020

Reads images from compressed RGB image topic
and stores the images in a user named folder
within the root directory. Image capture rate
is around 10fps
"""

import rospy
from std_msgs.msg import String
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import os
import time
import datetime

cat = "undefined"

# Read and store recieved images
def callback(data):
   bridge = CvBridge()
   cv_image = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
   time.sleep(0.1)
   timestamp = str(int(round(time.time()*1000))) + ".jpg"
   path = os.path.join(cat, timestamp)
   print(path)
   cv2.imwrite(path, cv_image)

# Init nodes and subscribers
def initFunct():
    image = CompressedImage()
    rospy.init_node('imagebrightness', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage,  callback, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    cat = raw_input("Enter category: ")
    if not os.path.exists(cat):
    	os.mkdir(cat)
    initFunct()
