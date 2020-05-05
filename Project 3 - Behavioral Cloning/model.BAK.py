# importing the necessary module to read the driving_log.csv file
import csv
import cv2

import numpy as np
import tensorflow as tf

from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

import os
from math import ceil
import matplotlib.pyplot as plt

from params import *

# importing the necessary layrs for a Lenet structure
from keras.layers import Conv2D, MaxPooling2D, AveragePooling2D, Cropping2D, Dropout

def generator(samples, batch_size = 32, augment_data = True):
    '''
    This function handles the reading, shuffling and pre-processing of image data and angles
    '''
    num_samples = len(samples)
    while 1: # looping forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            # Grabs a chunk of the data to process at one time
            batch_samples = samples[offset:offset + batch_size]

            images = []
            angles = []

            for batch_sample in batch_samples:
                dir = os.getcwd()
                image_path = dir + '/' + batch_sample[0]
                
                # Reading image data for center and adding it to the list
                image = cv2.imread(image_path)
                images.append(image)
                
                # Reading angle data and adding it to the list
                angle = float(batch_sample[3])
                angles.append(angle)

                # Data augmentation
                if augment_data:
                    images.append(np.fliplr(image))
                    angles.append(-angle)

            
            # converting data into numpy array as this is what the neural net uses
            x_train = np.array(images)
            y_train = np.array(angles)
            # Shuffling the data and then returning it
            yield shuffle(x_train, y_train)

# Reading the data from the driving_log.csv file

# creating an empty list that will contain all the lines
lines = []

# Opening the csv file
with open('driving_log.csv') as csvfile:
    # open the csv file object and storing as 'reader'
    reader = csv.reader(csvfile)
    # looping through every single line 
    for line in reader:
        # adding the liens we are reading the list 'lines'
        lines.append(line)

# Splitting data into traiing and validation
# TODO - Move into another file
# train_valid_split = 0.2



train_data, validation_data = train_test_split(lines, test_size = MODEL_CONFIG['split_data'])

batch_size = 32

# Getting the generator data for both training and validation
train_generator = generator(train_data, 32)
validation_denerator  = generator(validation_data, 32)

# Creating lists to hold both the images and the measurements
# images = []
# measurements = []

# for line in lines:
#     source_path = line[0]
#     # I believe the bottom three commented out lines will not be necessary.
#     # The reason for this is because we are not on any instance and our images are stored locally. 
#     # filename = source_path.split('/')[-1]

#     dir = os.getcwd()
#     source_path = dir + '/' + source_path
#     # current_path = '../data/' + source_path
#     # image = cv2.imread(current_path)
    
#     # Reading the image from the path found in the csvfile
#     # image = cv2.imread(source_path)
#     # images.append(image)

#     image = cv2.imread(source_path)
#     images.append(image)
    
#     # Reading the measurement from the file and storing it as a float. We cast to a float to ensure we don't lose data
#     measurement = float(line[3])
#     measurements.append(measurement)

#     # Augment the values below
#     images.append(np.fliplr(image))
#     measurements.append(-measurement)
    
# Converting our lists into numpy arrays as this is the requiered type that Keras requires
# x_train = np.array(images)
# y_train = np.array(measurements)

model = Sequential()

# TODO generalize the input shape at this layer in order to keep the code more modular. 
model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
# The layer below crops the image to get rid of the hood and environment imagery that is captures in the image. This will help the network generalize and focus on lanes.
# Removes the top 70 pixels and the bottom 25 pixels
model.add(Cropping2D(cropping=((70,25),(0,0))))

# Utilizing NVIDIA's approach for this the following layers. 5 convolutional layers followed 
model.add(Conv2D(24, (5, 5), activation="relu", strides=(2, 2)))
model.add(Conv2D(36, (5, 5), activation="relu", strides=(2, 2)))
model.add(Conv2D(48, (5, 5), activation="relu", strides=(2, 2)))
model.add(Conv2D(64, (3, 3), activation="relu"))
model.add(Conv2D(64, (3, 3), activation="relu")) 
model.add(Flatten())
model.add(Dropout(.2))
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))

# Choosing mse (mean quare error as the loss, and 'adam' as the optimizer)
model.compile(loss = 'mse' , optimizer = 'adam')

# Splitting out data so that 20% of it becomes the validation. 
# Setting the shuffle to true does what we think it does, it'll shuffle the data
# In addition it's good information to know what by default keras trains on 10 Epochs. We can change the amount as shown below

# model.fit(x_train, y_train, validation_split = 0.2, shuffle = True, epochs = 2)

history_object = model.fit_generator(train_generator, 
steps_per_epoch = ceil(len(train_data) / batch_size), 
validation_data= validation_denerator, 
validation_steps= ceil(len(validation_data) / batch_size), 
epochs = 7, verbose = 1)

# history_object = model.fit(x_train,y_train, validation_split = 0.2, shuffle= True,
#             epochs=11, verbose=1)

# saving the model as model.h5

name = "model.h5"
# tf.keras.experimental.export_saved_model(model, name)

model.save(name)

plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc = 'upper right')
plt.show()
