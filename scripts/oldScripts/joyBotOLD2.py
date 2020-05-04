#!/usr/bin/env python3
import math

import roslib;
import rospy

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float32
from beginner_tutorials.msg import Mvmt

import sys, select, termios, tty, time
import os
from sensor_msgs.msg import LaserScan
from beginner_tutorials.msg import Scaninfo
import csv

# Global angular variables 
curAngle = 0
targetAngle = 0
targetVel = 0

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
	
	pidSetPoint = curAngle + diff # Determine target angle 

	twist.angular.z = pidValue
	twist.linear.x = -targetVel/20000
	# Determine if it is safe to move in the direction of an obsticle
	if scaninfo.front < 1 and twist.linear.x > 0:
		twist.linear.x = 0
	if scaninfo.back < 1 and twist.linear.x < 0:
		twist.linear.x = 0
	print("Car orientation: " + str(curAngle))
	print("Gamepad orientation: " + str(targetAngle))
	# Write timing and angle data to CSV
	ms = int(round(time.time() * 1000))
	line = (str(curAngle) + ", " + str(targetAngle) + ", " + str(ms) + "\n")
	pub.publish(twist)


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
	print("PID interval is: " + (str(time.time() - pidPrevTime)))

	# Calculate PID terms
	pidError =(pidSetPoint - curAngle)
	pidProportional = 20 * pidError
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


def handleLidar(data):
	global scaninfo
	scaninfo.front = data.front
	scaninfo.back  = data.back

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
	sub4 = rospy.Subscriber('/clock', Imu, pidCallback)
	time.sleep(1)
	speed = rospy.get_param("~speed",3.0)
	turn = rospy.get_param("~turn", 3.0)
	rospy.spin()
