#!/usr/bin/env python3

"""
headset.py
Author: Samuel Love
Year: 2020

Launches viveOri as a subprocesses and captures output data
from the executable. Captured data is formattted as quaterion
type and published to  'headsetOri' topic
"""

import subprocess
import sys
import time
import os
import fcntl
import rospy

from geometry_msgs.msg import Quaternion

global p1
quaternion = Quaternion()


def main():
	global p1
	p1 = subprocess.Popen(["/home/samuel/pureOHMD/OpenHMD/build/vive_orientation/viveOri"], stdout=subprocess.PIPE, bufsize=-1)
	rospy.init_node('headsetOri')
	pub = rospy.Publisher('headsetOri', Quaternion, queue_size = 1)
	print("reached main")
	while(1):
		data = p1.stdout.readline()
		data = data.decode("utf-8")
		# Update appropriate axis based on data read from stdout
		if "Value at " in data:
			if "w" in data:
				quaternion.w = float(data[13:20])
			if "x" in data:
				quaternion.x = float(data[13:20])
			if "y" in data:
				quaternion.y = float(data[13:20])
			if "z" in data:
				quaternion.z = float(data[13:20])

		print("w = %f x = %f y = %f z = %f" % (quaternion.w, quaternion.x, quaternion.y, quaternion.z))
		pub.publish(quaternion)
	print("Exited while loop")

if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		# Send interrupt signal to viveOri process when program is interrupted
		print("killing process")
		os.system("killall -SIGINT viveOri")
		sys.exit(0)
