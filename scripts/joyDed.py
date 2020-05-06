#!/usr/bin/env python3

"""
joyDed.py
Author: Samuel Love
Year: 2020

Extracts input data from joystick controller, calculates
orientation angle and handles joystick deadzones 
publishes extracted data to 'joyTurn' topic
as 'mvmt' message structure
"""

# Inputs imports
from inputs import devices
from inputs import get_gamepad
import math

# ROS Imports
import roslib;
import rospy
from std_msgs.msg import Float32
from beginner_tutorials.msg import Mvmt


#import sys, select, termios, tty, time

# Initialise variables
curAngle = 0
targetAngle = 0
xVal = 0
yVal = 0
vel = 0

# Output all connected devices
for device in devices:
	print(device)


if __name__=="__main__":
	pub = rospy.Publisher('joyTurn', Mvmt, queue_size = 1)
	rospy.init_node('joyturn')
	mvmt = Mvmt()
	impulse = 2
	while(1):
		events = get_gamepad() # Get gamepad state
		if events != 0:
			for event in events:
			# Iterate through events object to find input states
				if(event.code == 'ABS_X'): # Left joystick x val
					xVal = event.state
					print(event.code, event.state)
				if(event.code == 'ABS_Y'): # Left joystick y val
					yVal = event.state
					print(event.code, event.state)
				if(event.code == 'ABS_RY'): # Right joystick y val
					vel = event.state
				if(event.code == 'BTN_EAST'): # -1 Impulse on b button
					if impulse == -1:
						impulse = 2
					else:
						impulse = -1
				if(event.code == 'BTN_WEST'): # 0 Impulse on y button
					if impulse == 0:
						impulse = 2
					else:
						impulse = 0
				print(event.code)
		if impulse == 2: # If impulse if not present calc angle
			if (yVal == 0 and xVal == 0):
				targetAngle = 0 # Handle error condition
			else:
				targetAngle = math.atan2(-yVal, xVal)
		else: # Multiply by impulse to get 0 or 1
			targetAngle = math.pi*impulse
		mvmt.angle = targetAngle
		if abs(vel) > 3000: # Account for deadzone in controller
			mvmt.vel = vel
		else:
			mvmt.vel = 0
		print("Joystick angle is: " + str(mvmt.angle) + " rads")
		print("Joystick velocity is: " + str(mvmt.vel))
		pub.publish(mvmt)
	print("Main loop ended")
