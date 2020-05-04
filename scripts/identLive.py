#!/usr/bin/env python3

"""
identLive.py
Author: Samuel Love
Year: 2020

Performs live netowkr predictions and overlays 
classification and LiDAR ranging information to 
input image feed. Processed image is converted and
republished to the 'procimage' topic
"""

# Keras Imports
import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Activation
from keras.layers import Conv2D, MaxPooling2D
from keras import optimizers
from keras.optimizers import SGD
from keras.models import load_model
from tensorflow.python.client import device_lib
from keras.utils import plot_model
import tensorflow
from tensorflow.keras.models import model_from_json

print(device_lib.list_local_devices())

# General Python Imports
import numpy as np
import os
import cv2
import datetime
import matplotlib.pyplot as plt
import math

# Rospy Imports
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from beginner_tutorials.msg import Scaninfo


# Configuration variables
savedModel = True
categories = 4
model = tensorflow.keras.models.load_model('/home/samuel/newProject/src/beginner_tutorials/scripts/simNetwork.h5') # Change as appropriate
testDataLoc = '/home/samuel/newProject/src/beginner_tutorials/scripts/procImages'
dataCount = 0


#Sensor variables
distances = [0, 0]

# Function for printing overlay data onto images
def printOverlay(predictions, testImages):
    global dataCount
    global distances
    dirs = [ f.name for f in os.scandir(testDataLoc) if f.is_dir() ]
    dirs = sorted(dirs)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8
    #Overlay classification text onto images
    for i in range(len(testImages)):
        for j in range(categories):
            catTextPos = (10, 350 + 30*j)
            scoreTextPos = (140, 350 + 30*j)
            line = '{:<12}  {:>5}'.format(dirs[j], str(round(predictions[i][j], 5)))
            fontColour = (0, 0, 0)
            thickness = 2
            cv2.putText(testImages[i], dirs[j], catTextPos, font, fontScale, fontColour, thickness)
            cv2.putText(testImages[i], str(round(100*predictions[i][j], 5)) + "%", scoreTextPos, font, fontScale, fontColour, thickness)
            fontColour = (255, 255, 255)
            thickness = 1
            cv2.putText(testImages[i], str(round(100*predictions[i][j], 5)) + "%", scoreTextPos, font, fontScale, fontColour, thickness)
            cv2.putText(testImages[i], dirs[j], catTextPos, font, fontScale, fontColour, thickness)

        #Overlay LiDAR text onto images
        for c in range(2):
            lidTextPos = (470, 30 + 70*c)
            fontColour = (0, 0, 0)
            thickness = 2
            cv2.putText(testImages[i], str(round(distances[c], 5)), lidTextPos, font, fontScale, fontColour, thickness)
            thickness = 1
            fontColour = (255, 255, 255)
            cv2.putText(testImages[i], str(round(distances[c], 5)), lidTextPos, font, fontScale, fontColour, thickness)

        #Draw indication arrows onto image
        startPoints = [50, 20]
        endPoints = [20, 50]
        for c in range(2):
            arrowStart = (600, startPoints[c] + 70*c)
            arrowEnd = (600, endPoints[c] + 70*c)
            arrowColour = (0, 0, 255)
            arrowThickness = 3
            arrowTipLength = 0.5
            cv2.arrowedLine(testImages[i], arrowStart, arrowEnd, arrowColour, arrowThickness, tipLength = 0.5)
        #Draw indicator rectangles onto image
        for c in range(2):
            topLeft = (470, 50 + 70*c)
            bottomRight = (570, 60 + 70*c)
            thickness = -1
            colour = (0, 255, 0)
            cv2.rectangle(testImages[i], topLeft, bottomRight, colour, thickness)
	#Fill indicator rectangles to indicate distance
        for c in range(2):
            topLeft = (470, 50 + 70*c)
            distancePixelShift = 100 - int((distances[c] / 12) * 100)
            bottomRight = (470 + distancePixelShift , 60 + 70*c)
            thickness = -1
            colour = (255, 0, 0)
            cv2.rectangle(testImages[i], topLeft, bottomRight, colour, thickness)

        # Save image
        savePath = '/home/samuel/Desktop/DataImages'
        fileName = "evaluated" + str(dataCount) + ".jpg"
        dataCount += 1
        cv2.imwrite(os.path.join(savePath, fileName), testImages[i])
    return testImages[i]

# Normalise predictions within range 0 to 1
def normalisePredictions(predictions):
    for i in range(len(predictions)):
        normFactor = np.sum(predictions[i])
        for j in range(len(predictions[i])):
            predictions[i][j] = predictions[i][j] / normFactor
    return predictions


# Convert Numpy array to imgmsg format and republish
def convAndPub(image):
    global pub
    bridge = CvBridge()
    image = image.astype(np.uint8)
    image_message = bridge.cv2_to_imgmsg(image, "passthrough")
    image_message.encoding = "rgb8"
    print(image_message.encoding)
    pub.publish(image_message)

# Extact LiDAR data from topic
def getLidarData(data):
    global distances
    distances[0] = data.front
    distances[1] = data.back

# Normalise image Numpy array
def normaliseData(data):
	return data / 255

# High level function calls for data manipulation, predictions and publishing
def callback(data):
    global model
    global pub
    bridge = CvBridge()
    # Convert and normalise input image data
    cv_image = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
    cv_image2 = np.zeros((1, 480, 640, 3))
    cv_image2[0] = cv_image
    cv_image = cv_image2
    cv_imageBackup = cv_image
    cv_image = normaliseData(cv_image)
    # Calculate and normalise predictions
    predictions = model.predict(cv_image)
    predictions = normalisePredictions(predictions)
    print(predictions)
    # Overlay data onto processed image
    processedImage = printOverlay(predictions, cv_imageBackup)
    # Convert processed image to imgmsg and republish
    convAndPub(processedImage)


image = CompressedImage()
rospy.init_node('imagepredictions')
rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage,  callback, queue_size=1)
rospy.Subscriber('/scanDist', Scaninfo,  getLidarData, queue_size=5)
pub = rospy.Publisher('procimage', Image, queue_size=1)
rospy.spin()
