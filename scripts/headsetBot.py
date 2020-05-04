#!/usr/bin/env python3

"""
headsetBot.py
Author: Samuel Love 
Year: 2020

Reads IMU data from ROSBot and headset 
and uses this information to calculate
angular and linear velocity of ROSBot
Includes simple collision avoidence
system
"""

import math

import roslib;
import rospy

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float32
from beginner_tutorials.msg import Mvmt
from beginner_tutorials.msg import Scaninfo

import sys, select, termios, tty, time
import os

import csv

# Global message types
imu = Imu()
quaternion = Quaternion()
twist = Twist()
scaninfo = Scaninfo()

# Global angular variables
curAngle = 0
targetAngle = 0
targetVel = 0
prevTime = time.time()

# Global PID parameters
pidPrevError = 0
pidIntegral = 0
pidPrevTime = 0
pidSampleTime = 0.01
pidValue = 0
pidSetPoint = 0

# Convert quaternion data to Euler angles
def quatToEuler(data, angleType):
	q0 = data.w
	q1 = data.x
	q2 = data.y
	q3 = data.z
	if angleType == "phi":
		return (math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))
	if angleType == "theta":
		return (math.asin(2*(q0*q2 - q3*q1)))
	if angleType == "omega":
		return (math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))

# Store LiDAR data
def handleLidar(data):
	global scaninfo
	scaninfo.front = data.front
	scaninfo.back = data.back

# Calculate angular differences and apply PID output
def callback(data):
	global targetAngle
	global targetVel
	global pub
	global fileName
	global scaninfo
	global pidSetPoint
	global curAngle
	global pidValue
	global prevTime
	diff = 0
	curAngle = quatToEuler(data.orientation, "phi")
	diff = targetAngle - curAngle
	if diff > math.pi:
		diff += -2*math.pi
	elif diff < -math.pi:
		diff += 2*math.pi
	else:
		diff = (targetAngle - curAngle)

	pidSetPoint = curAngle + diff # Determine target angle
	control = pidCallback(0)
	print("Pid output: " + str(control))
	twist.angular.z = control
	twist.linear.x = -targetVel
	if scaninfo.front < 1 and twist.linear.x > 0:
		twist.linear.x = 0
	if scaninfo.back < 1 and twist.linear.x < 0:
		twist.linear.x = 0

	print("ROSBot Orientation: " + str(curAngle))
	print("Vive Z axis orientation: " + str(targetAngle))
	pub.publish(twist)


def callback2(data):
	global targetAngle
	global targetVel
	# Unproven code
	#x = data.x
	#y = data.y
	#z = data.z
	#w = data.w
	#data.x = w
	#data.w = x
	# Untested code
	w = data.z
	x = data.y
	y = data.x
	z = data.w
	targetAngle = -quatToEuler(data, "phi")
	targetVel = quatToEuler(data, "theta")
	if abs(targetVel) < 0.2:
		targetVel = 0


# Calculate the constants for Pid control
def pidCallback(data):
	global pidValue
	global targetAngle
	global curAngle
	global pidIntegral
	global pidPrevError
	global pidPrevTime
	global pidSampleTime
	global pidSetPoint
	pidInterval = time.time() - pidPrevTime
	# Account for processing margins by waiting until present delay is met
	if pidSampleTime > pidInterval:
		time.sleep(pidSampleTime - pidInterval)
	print("PID interval is" + (str(time.time() - pidPrevTime)))

	# Calculate PID terms
	pidError = (pidSetPoint - curAngle)
	pidProportional = 15 * pidError
	pidIntegral = 0 * (pidIntegral + (pidError * pidInterval))
	pidDerivitive = 0.6 * ((pidError - pidPrevError) / pidInterval)

	# Prevent integral windup by capping PID output at a suitable value (30)
	tempPidValue = pidProportional + pidIntegral + pidDerivitive
	if abs(tempPidValue) > 30:
		if tempPidValue < 0:
			pidValue = -30
		else:
			pidValue = 30
	else:
		pidValue = tempPidValue
	# Update function values for next iteration
	pidPrevError = pidError
	pidPrevTime = time.time()
	return pidValue


xVal = 2
yVal = 2

if __name__=="__main__":
	rospy.init_node('botHeadsetControl')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	sub = rospy.Subscriber('/imu', Imu, callback)
	sub2 = rospy.Subscriber('headsetOri', Quaternion, callback2)
	sub3 = rospy.Subscriber('scanDist', Scaninfo, handleLidar)
	time.sleep(1)
	speed = rospy.get_param("~speed",3.0)
	turn = rospy.get_param("~turn", 3.0)
	rospy.spin()
