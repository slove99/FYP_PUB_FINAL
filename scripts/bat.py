#!/usr/bin/env python

"""
bat.py
Author: Samuel Love
Year: 2020

Subscribes to battery topic and prints a calculated
battery percentage to the console window
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

def callback(data):
    print("Battery level is: %f volts" % (data.voltage))
    batteryPercent = ((data.voltage - 9)/3.6)*100
    print("Battery percentage is: %f percent" % (batteryPercent))

def listener():
    batterystate = BatteryState()
    rospy.init_node('batpercent', anonymous=True)
    rospy.Subscriber('battery', BatteryState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()