#!/usr/bin/env python

#General imports
from __future__ import print_function
import roslib;
import rospy

#Motor control imports
from geometry_msgs.msg import Twist
import sys, select, termios, tty, time

#Depth processing imports
from std_msgs.msg import String
import sys
import cv2
import time
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import math


moveBindings = {
        'w':(1,0,0,0),
        's':(-1,0,0,0),
 	'a':(0,0,0,1),
	'd':(0,0,0,-1),
        'q':(0,0,0,0),
    }

distance = 0 
printcounter = 0

def getKey():
    key = 'q'
    tty.setraw(sys.stdin.fileno())
    if select.select([sys.stdin,],[],[],0.1)[0]:
    	key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   # print("Key pressed")
    return key



def callback(data):
   global distance
   global printcounter
   tot = 0
   offset = 330
   for i in range(60):
	index = (i+offset) % len(data.ranges)
        if not(math.isinf(data.ranges[index])):
   		tot += data.ranges[index]
   distance = tot/60
   if ((printcounter + 1) % 5) == 0:
   	print(str(distance) + "\n")
   printcounter += 1
   sys.stdout.flush()



if __name__=="__main__":
    global distance
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 3)
    rospy.init_node('teleop_twist_keyboard')
    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        while(1):
            key = getKey()
	    #print("Key is: " + key + "\n")
            if (key in moveBindings.keys()) and (distance > 0.4 or key != 'w'):
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

	    else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break


 	    twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

	    laser = LaserScan()
   	    rospy.Subscriber('/scan', LaserScan,  callback, queue_size=1) 
    except Exception as e:
    	print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
