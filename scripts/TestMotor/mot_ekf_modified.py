#!/usr/bin/env python

#Imports
from __future__ import print_function

import roslib;
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty, time

moveBindings = {
        'w':(1,0,0,0),
        's':(-1,0,0,0),
 	'a':(0,0,0,1),
	'd':(0,0,0,-1),
        'q':(0,0,0,0),
    }


def getKey():
    key = 'q'
    tty.setraw(sys.stdin.fileno())
    if select.select([sys.stdin,],[],[],0.1)[0]:
    	key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print("Key pressed")
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
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
	    print("Key is: " + key + "\n")
            if key in moveBindings.keys():
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

    except Exception as e:
    	print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
