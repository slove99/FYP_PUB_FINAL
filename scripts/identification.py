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

from keras.layers import GaussianNoise

print(device_lib.list_local_devices())

# General Imports
import numpy as np
import os
import cv2
import datetime
import matplotlib.pyplot as plt

# Configuration variables
testDataLoc = 'testingData3'
trainDataLoc = 'trainingDataREAL'
savedModel = False
categories = 4

def genTrainAndCat(folder, categories):
    """ Function to generate training data arrays and category arrays
        INPUTS: 'folder' training data folder
                'categories' number of training categories
        OUTPUTS: 'imageArray' array of image training data
                 'imageClasses' array of classes of image training data
    """
    allFiles = list(os.walk(folder))
    fileCount = 0

    # Count number of training files for preallocating array size
    for root, dirs, files in os.walk(folder):
        fileCount += len(files)

    # Extract category names from training data folder names
    catData = [ f.path for f in os.scandir(folder) if f.is_dir() ]
    catData = sorted(catData)

    # Create storage structures for training data and their categories
    imageArray = np.zeros((fileCount, 480, 640, 3))
    imageClasses = np.zeros((fileCount, categories))
    offset = 0

    print("Found %d categories" % len(catData))
    for i in range(len(catData)): # Iterate through each category folder
        fileList = os.listdir(catData[i])
        imgNum = len(fileList)
        print("Found category %s with %d files " % (catData[i], imgNum))
        # Iterate through each training image in the category folder, add data and category to arrays
        for count, image in enumerate(fileList):
            imageArray[count + offset] = cv2.imread(os.path.join(catData[i], fileList[count]))
            imageClasses[count + offset][i] = 1
        offset += len(allFiles[i + 1][2])
    return [imageArray, imageClasses]

def normaliseData(x):
    x = x / 255
    return(x)

def createModel():
    """ Function to create network structure
        INPUTS: N/A
        OUTPUTS: Network model
    """
    model = Sequential()
    model.add(Conv2D(32, (2, 2), input_shape=(480, 640, 3))) #32 output filter, convolving on a 2x2 area
    model.add(GaussianNoise(0.05))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2))) # Scale down the image by a factor of 2 in x and y

    model.add(Conv2D(64, (2, 2)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(32, (2, 2)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(4, 4)))

    # Create fully connected output
    model.add(Flatten())
    model.add(Dense(32))
    model.add(Activation('relu'))
    model.add(Dropout(0.5))
    model.add(Dense(categories))
    model.add(Activation('softmax'))
    return model

def shuffleTrainAndCat(x_train, y_train):
    """ Function to shuffle training and categories in the same manner
          INPUTS: 'x_train' training data array
                  'y_train' category array
          OUTPUTS: 'x_train' shuffled training data array
                   'x_train' shuffled training data array
    """
    shuffleSeed = np.random.get_state()
    np.random.shuffle(x_train)
    np.random.set_state(shuffleSeed)
    np.random.shuffle(y_train)
    return x_train, y_train

def saveModel(model):
    """ Function to save network model to .h5 file"""
    currentDT = datetime.datetime.now()
    dateString = str(currentDT.strftime("%Y_%m_%d_%H_%M_%S"))
    model.save(dateString + ".h5")

def plotModel(model, history):
    """ Function to plot accuracy and loss graphs
    INPUTS: 'model' network model
            'history' training history object for network model
    OUTPUTS: 'Accuracy graph' graph of accuracy over epochs
             'Loss graph' graph of loss over epochs
    """
    # Plot training & validation accuracy values
    fig, (accPlot, lossPlot) = plt.subplots(nrows=2, ncols=1, sharex=False, sharey=False)
    # Plot accuracy graph
    accPlot.plot(history.history['accuracy'])
    accPlot.plot(history.history['val_accuracy'])
    accPlot.set_title('Model accuracy')
    accPlot.set_ylabel('Accuracy')
    accPlot.set_xlabel('Epoch')
    accPlot.legend(['Train', 'Test'], loc='upper left')

    # Plot loss graph
    lossPlot.plot(history.history['loss'])
    lossPlot.plot(history.history['val_loss'])
    lossPlot.set_title('Model loss')
    lossPlot.set_ylabel('Loss')
    lossPlot.set_xlabel('Epoch')
    lossPlot.legend(['Train', 'Test'], loc='upper left')
    plt.savefig('testloss.png')
    plt.show()

def calcPredictAcc(predictions):
    """ Function show the accuracy of the network on unseen data as a percentage
        INPUTS: 'predictions' predictions array
        OUTPUTS: Percentage accuracy of predictions
    """
    numCorrect = 0
    for i in range(len(predictions)):
        testValue = list(y_test[i]).index(1)
        predValue = list(predictions[i]).index(np.amax(predictions[i]))
        print("Testing data %d: Known category = %d, Predicted category = %d" % (i, testValue, predValue))
        if (testValue == predValue):
            numCorrect += 1
    return ((numCorrect / len(predictions)) * 100)

def normalisePredictions(predictions):
    """ Function to normalise output of prediction function on network
           INPUTS: 'predictions' predictions array
           OUTPUTS: 'predictions'  Normalised predictions array
    """
    for i in range(len(predictions)):
        normFactor = np.sum(predictions[i])
        for j in range(len(predictions[i])):
            predictions[i][j] = predictions[i][j] / normFactor
    return predictions

def printOverlay(predictions, testImages):
    """ Function to overlay predictions text onto input images
           INPUTS: 'predictions' predictions array
                   'testImages' image array to be processed
           OUTPUTS: Processed images to write to disk
    """
    dirs = [ f.name for f in os.scandir(testDataLoc) if f.is_dir() ]
    dirs = sorted(dirs)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8
    for i in range(len(testImages)):
        for j in range(categories):
            catTextPos = (10, 350 + 30*j)
            scoreTextPos = (140, 350 + 30*j)
            #line = dirs[j] + " = " + str(round(predictions[i][j], 5)) + "%"
            line = '{:<12}  {:>5}'.format(dirs[j], str(round(predictions[i][j], 5)))
            fontColor = (0, 0, 0)
            thickness = 2
            cv2.putText(testImages[i], dirs[j], catTextPos, font, fontScale, fontColor, thickness)
            cv2.putText(testImages[i], str(round(100*predictions[i][j], 5)) + "%", scoreTextPos, font, fontScale, fontColor, thickness)
            fontColor = (255, 255, 255)
            thickness = 1
            cv2.putText(testImages[i], str(round(100*predictions[i][j], 5)) + "%", scoreTextPos, font, fontScale, fontColor, thickness)
            cv2.putText(testImages[i], dirs[j], catTextPos, font, fontScale, fontColor, thickness)
        # Save image
        fileName = "evaluated" + str(i) + ".jpg"
        cv2.imwrite(fileName, testImages[i])


if savedModel == False:
    # Build training and testing arrays and categories
    x_train, y_train = genTrainAndCat(trainDataLoc, categories)
    # Normalise training data
    x_train = normaliseData(x_train)
    # Shuffle training data
    x_train, y_train = shuffleTrainAndCat(x_train, y_train)
    # Create model
    model = createModel()

    model.summary()
    model.compile(optimizer='Adam', loss='categorical_crossentropy', metrics=['accuracy'])
    history = model.fit(x_train, y_train, batch_size=5, epochs=6, validation_split=0.25, verbose=1) #batch 5 epochs 5
    saveModel(model) # Save model
    plotModel(model, history)


if savedModel == True:
    model = load_model('simNetwork.h5')


x_test, y_test = genTrainAndCat(testDataLoc, categories)

x_testBackup = x_test
x_test = normaliseData(x_test)

predictions = model.predict(x_test)
predictions = normalisePredictions(predictions)

percentCorrect = calcPredictAcc(predictions)

printOverlay(predictions, x_testBackup)

print(percentCorrect)


