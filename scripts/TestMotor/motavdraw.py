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
from sensor_msgs.msg import CompressedImage
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
   imageSum = 0
   step = 20
   bridge = CvBridge()
   cv_image = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
   for i in range(0, cv_image.shape[0], step):
       for j in range(0, cv_image.shape[1], step):
           if cv_image[i][j] == cv_image[i][j]:
                imageSum += cv_image[i][j] #cv_image
                #print("it's not nan")
                #print(cv_image[i][j])
           #else:
                #print("it is nan")
           #print("i is " + str(i))
   imageAvg = (imageSum*step*step) / (cv_image.shape[0]*cv_image.shape[1])
  # print("IMAGE AVERAGE IS:  ")
  # print(imageAvg)
   distance = imageAvg
   if ((printcounter + 1) % 200) == 0:
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
            if (key in moveBindings.keys()) and (distance > 40 or key != 'w'):
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

	    image = CompressedImage()
   	    rospy.Subscriber('/camera/depth_registered/image_raw/compressed',Image, callback, queue_size=1) 
    except Exception as e:
    	print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
