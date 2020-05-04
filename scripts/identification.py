#!/usr/bin/env python3
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


print(device_lib.list_local_devices())


import numpy as np
import os
import cv2
import datetime
import matplotlib.pyplot as plt

#Configuration variables
savedModel = True
categories = 4

def genTrainAndCat(folder, categories):
    allFiles = list(os.walk(folder))
    fileCount = 0

    for root, dirs, files in os.walk(folder):
        fileCount += len(files)

    catData = [ f.path for f in os.scandir(folder) if f.is_dir() ]
    catData = sorted(catData)
    #Create storage structures for training data and their categories
    imageArray = np.zeros((fileCount, 480, 640, 3))
    imageClasses = np.zeros((fileCount, categories))
    offset = 0

    print("Found %d categories" % len(catData))
    for i in range(len(catData)): #Iterate through each category folder
        #path = os.path.join('bag')
        fileList = os.listdir(catData[i])
        imgNum = len(fileList)
        print("Found category %s with %d files " % (catData[i], imgNum))
        for count, image in enumerate(fileList):
            imageArray[count + offset] = cv2.imread(os.path.join(catData[i], fileList[count]))
            imageClasses[count + offset][i] = 1
        offset += len(allFiles[i + 1][2])
    return [imageArray, imageClasses]

def normaliseData(x):
    x = x / 255
    return(x)

def createModel():
    model = Sequential()
    model.add(Conv2D(32, (2, 2), input_shape=(480, 640, 3))) #32 output filter, convolving on a 2x2 area
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2))) #scale down the image by a factor of 2 in x and y

    model.add(Conv2D(64, (2, 2)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(32, (2, 2)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(4, 4)))

    model.add(Flatten())
    model.add(Dense(32))
    model.add(Activation('relu'))
    model.add(Dropout(0.25))
    model.add(Dense(4))
    model.add(Activation('relu'))
    return model

def shuffleTrainAndCat(x_train, y_train):
    shuffleSeed = np.random.get_state()
    np.random.shuffle(x_train)
    np.random.set_state(shuffleSeed)
    np.random.shuffle(y_train)
    return x_train, y_train

def saveModel(model):
    currentDT = datetime.datetime.now()
    dateString = str(currentDT.strftime("%Y_%m_%d_%H_%M_%S"))
    model.save(dateString + ".h5")

def plotModel(model, history):
    # Plot training & validation accuracy values
    plt.plot(history.history['accuracy'])
    plt.plot(history.history['val_accuracy'])
    plt.title('Model accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')
    plt.show()

if savedModel == False:
    # Build training and testing arrays and categories
    x_train, y_train = genTrainAndCat('trainingData', categories)

    # Normalise training data
    x_train = normaliseData(x_train)

    # Shuffle training data
    x_train, y_train = shuffleTrainAndCat(x_train, y_train)

    # Create model
    model = createModel()

    model.summary()
    model.compile(optimizer='Adam', loss='categorical_crossentropy', metrics=['accuracy'])
    history = model.fit(x_train, y_train, batch_size=5, epochs=15, validation_split=0.1, verbose=1)
    saveModel(model) # Save model
    plotModel(model, history)

print("Wait here")


if savedModel == True:
    model = load_model('100percentNew.h5')

x_test, y_test = genTrainAndCat('testingData', categories)
x_test = normaliseData(x_test)

predictions = model.predict(x_test)

numCorrect = 0

for i in range(len(predictions)):
    testValue = list(y_test[i]).index(1)
    predValue = list(predictions[i]).index(np.amax(predictions[i]))
    print("Testing data %d: Known category = %d, Predicted category = %d" % (i, testValue, predValue))
    if (testValue == predValue):
        numCorrect += 1
print("Percentage correct is: %f" % (numCorrect / len(predictions) * 100))



#score = model.evaluate(x_test, y_test, batch_size=1)
# Generate dummy data
#x_train = imageArray #The training data itself
#y_train = keras.utils.to_categorical(imageClasses, num_classes=4) #The ordered labels to all the training data in the set
#x_test = np.random.random((20, 100, 100, 3))
#y_test = keras.utils.to_categorical(np.random.randint(10, size=(20, 1)), num_classes=10)
