#!/usr/bin/env python

"""
lidar.py
Author: Samuel Love
Year: 2020

Calculates minimum LiDAR distance over a specified
angular range from the front and rear of the ROSBot
Calculated values are published on the scanDist topic
as message type Scaninfo
"""

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from beginner_tutorials.msg import Scaninfo

from cv_bridge import CvBridge, CvBridgeError
import math

global pub
scaninfo = Scaninfo()

# Print LiDAR parameters to console
def getLidarParams(data):
    print("Lidar range is %fm to %fm" % (data.range_min, data.range_max))
    print("Lidar angle increment is %f" % data.angle_increment)
    print("Lidar array size is %d" % len(data.ranges))

def getLidarData(data):
    #getLidarParams(data)
    global pub
    angle = 20
    angleCount = 0
    min = data.range_max
    apt = len(data.ranges)
    # Calculate offset range
    offset = [(apt/2) - angle, apt - angle]
    distances = [0, 0]
    for a in range(2): # Iterate through front and rear detection
        curOffset = offset[a]
        for i in range(2*angle):
            index = (i+curOffset) % len(data.ranges)
            if not(math.isinf(data.ranges[index])) and (data.ranges[index] > data.range_min):
                angleCount += 1
		if data.ranges[index] < min: # Update distance if a new min value is found
			min = data.ranges[index]
        distances[a] = min
        min = data.range_max
    scaninfo.back = distances[0]
    scaninfo.front = distances[1]
    print("Mean distance from front +- %d degrees is: %f" % (angle, distances[0]))
    print("Mean distance from read +- %d degrees is: %f" % (angle, distances[1]))
    pub.publish(scaninfo)

def main():
    global pub
    laserscan = LaserScan()
    rospy.init_node('lidarDepthData')
    rospy.Subscriber('/scan', LaserScan,  getLidarData, queue_size=5)
    pub = rospy.Publisher('/scanDist', Scaninfo, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    main()
