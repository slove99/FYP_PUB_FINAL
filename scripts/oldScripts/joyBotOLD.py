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

curAngle = 0
targetAngle = 0
targetVel = 0
imu = Imu()
quaternion = Quaternion()
twist = Twist()


def callback(data):
	global targetAngle
	global targetVel
	global pub
	diff = 0
	#print("callback reached")
	#if data.theta < 0:
	#	curAngle = data.theta + 2*math.pi
	#else:
	#curAngle = data.orientation.z*math.pi
	#curAngle = curAngle / (abs(data.orientation.w) / data.orientation.w)
	q0 = data.orientation.w
	q1 = data.orientation.x
	q2 = data.orientation.y
	q3 = data.orientation.z
	#curAngle = math.atan((2*(q0*q1 + q2*q3)) / (1-2*(q1**2 + q2**2)))
	#curAngle = math.asin(2*(q0*q2 - q3*q1))
	curAngle = 2 * math.atan( (2*(q0*q3 + q1*q2)) / (1-2*(q2**2 + q3**2) ) )
	diff = (targetAngle - curAngle)
	if diff > math.pi:
		diff += -(2*math.pi)
	elif diff < -math.pi:
		diff += 2*math.pi
	else:
		diff = 1*(targetAngle - curAngle)
	twist.angular.z = diff * 10
	twist.linear.x = -targetVel/20000
	#twist.linear. = -targetVel/10000
	print("Car orientation: " + str(curAngle))
	print("Gamepad orientation: " + str(targetAngle))
	pub.publish(twist)


def callback2(data):
	global targetAngle
	global targetVel
	#if data.data < 0:
	#	targetAngle = data.data + 2*math.pi
	#else:
	targetAngle = data.angle
	targetVel = data.vel

xVal = 2
yVal = 2

if __name__=="__main__":
	rospy.init_node('botControl')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	sub = rospy.Subscriber('/imu', Imu, callback)
	sub2 = rospy.Subscriber('joyTurn', Mvmt, callback2)
	time.sleep(1)
	speed = rospy.get_param("~speed",3.0)
	turn = rospy.get_param("~turn", 3.0)
	#print("I got to main")
	rospy.spin()
