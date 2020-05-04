#!/usr/bin/env python3

"""
joyBot.py
Author: Samuel Love
Year: 2020

Reads IMU data from ROSBot and angular information from joystick
Calculates angular and linear velocity of ROSBot. Performs
simple collision avoidence.
"""

import math

import roslib;
import rospy

# ROS Message Imports
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from beginner_tutorials.msg import Mvmt
from sensor_msgs.msg import LaserScan
from beginner_tutorials.msg import Scaninfo

import time
import os

# Global angular variables
curAngle = 0
targetAngle = 0
targetVel = 0
prevTime = time.time()

# Global message types
imu = Imu()
quaternion = Quaternion()
twist = Twist()
scaninfo = Scaninfo()

# Global PID parameters
pidPrevError = 0
pidIntegral = 0
pidPrevTime = 0
pidSampleTime = 0.01
pidValue = 0
pidSetPoint = 0

def movementCallback(data):
	global targetAngle
	global targetVel
	global pub
	global scaninfo
	global curAngle
	global pidValue
	global prevTime
	global pidSetPoint
	diff = 0
	# Calculate current angle from quadrature data
	q0 = data.orientation.w
	q1 = data.orientation.x
	q2 = data.orientation.y
	q3 = data.orientation.z
	curAngle = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
	# Calculate different between target and current angles account for
	# +-pi transition
	diff = targetAngle - curAngle
	if diff > math.pi:
		diff += -(2*math.pi)
	elif diff < -math.pi:
		diff += 2*math.pi
	else:
		diff = 1*(targetAngle - curAngle)
	pidSetPoint = curAngle + diff
	pidValue = pidCallback(0) # Calculate PID for iteration
	twist.angular.z = pidValue
	twist.linear.x = -targetVel/20000
	# Determine if it is safe to move in the direction of an obsticle
	if scaninfo.front < 1 and twist.linear.x > 0:
		twist.linear.x = 0
	if scaninfo.back < 1 and twist.linear.x < 0:
		twist.linear.x = 0
	print("Car orientation: " + str(curAngle))
	print("Gamepad orientation: " + str(targetAngle))
	print("\n")
	pub.publish(twist)


# Calculate the constants for PID control
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
	# Account for processing margins by waiting until preset delay is met
	# However function is event based so is limited at 0.1s interval
	if pidSampleTime > pidInterval:
		time.sleep(pidSampleTime - pidInterval)
	print("PID interval is " + (str(time.time() - pidPrevTime)))

	# Calculate PID terms
	kp = 20
	ki = 0
	kd = 0.6
	pidError = (pidSetPoint - curAngle)
	pidProportional = pidError
	pidIntegral += (pidError * pidSampleTime)
	pidDerivitive =  ((pidError - pidPrevError) / pidSampleTime)
	print("Derivitive is " + str(kd * pidDerivitive))
	print("Integral is " + str(ki * pidIntegral))
	print("Proportional is " + str(kp * pidProportional))
	tempPidValue = kp*pidProportional + ki*pidIntegral + kd*pidDerivitive
	# Prevent integral windup by capping PID output at a suitable value (30)
	if abs(tempPidValue) > 25:
		if tempPidValue < 0:
			pidValue = -25
		else:
			pidValue = 25
	else:
		pidValue = tempPidValue
	print("Error is " + str(pidError))
	# Update function values for next iteration
	pidPrevError = pidError
	pidPrevTime = time.time()
	return(pidValue)

# Store LiDAR data
def handleLidar(data):
	global scaninfo
	scaninfo.front = data.front
	scaninfo.back  = data.back

# Store joystick data
def joyCallback(data):
	global targetAngle
	global targetVel
	targetAngle = data.angle
	targetVel = data.vel

xVal = 2
yVal = 2

if __name__=="__main__":
	rospy.init_node('botJoyControl')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	sub = rospy.Subscriber('/imu', Imu, movementCallback)
	sub2 = rospy.Subscriber('joyTurn', Mvmt, joyCallback)
	sub3 = rospy.Subscriber('scanDist', Scaninfo, handleLidar)
	time.sleep(1)
	speed = rospy.get_param("~speed",3.0)
	turn = rospy.get_param("~turn", 3.0)
	rospy.spin()
