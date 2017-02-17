import os
import csv
import cv2
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten, SpatialDropout2D, ELU
from keras.layers import Convolution2D, MaxPooling2D, Cropping2D
from keras.layers.core import Lambda

from keras.optimizers import SGD, Adam, RMSprop
from keras.utils import np_utils

from keras.callbacks import ModelCheckpoint

from keras.models import model_from_json



## 1. Prepare and create generator

# Save filepaths of images to `samples` to load into generator    
samples = []

def add_to_samples(csv_filepath, samples):
    with open(csv_filepath) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)
    return samples

samples = add_to_samples('data-udacity/driving_log.csv', samples)

samples = add_to_samples('data-recovery-annie/driving_log.csv', samples) # header already removed

# Remove header
samples = samples[1:]
 
print("Samples: ", len(samples))        

# Split samples into training and validation sets to reduce overfitting
train_samples, validation_samples = train_test_split(samples, test_size=0.1)

def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = './data-udacity/'+batch_sample[0]
                center_image = mpimg.imread(name)
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)

            X_train = np.array(images)
            y_train = np.array(angles)
            
            yield shuffle(X_train, y_train)

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)


## 2. Data Preprocessing functions

def resize_comma(image):
    import tensorflow as tf  # This import is required here otherwise the model cannot be loaded in drive.py
    return tf.image.resize_images(image, 40, 160)


## 3. Model (data preprocessing incorporated into model)

# Model adapted from Comma.ai model

model = Sequential()

# Crop 70 pixels from the top of the image and 25 from the bottom
model.add(Cropping2D(cropping=((70, 25), (0, 0)),
                     dim_ordering='tf', # default
                     input_shape=(160, 320, 3)))

# Resize the data
model.add(Lambda(resize_comma))

# Normalise the data
model.add(Lambda(lambda x: (x/255.0) - 0.5))

# Conv layer 1
model.add(Convolution2D(16, 8, 8, subsample=(4, 4), border_mode="same"))
model.add(ELU())

# Conv layer 2
model.add(Convolution2D(32, 5, 5, subsample=(2, 2), border_mode="same"))
model.add(ELU())

# Conv layer 3
model.add(Convolution2D(64, 5, 5, subsample=(2, 2), border_mode="same"))

model.add(Flatten())
model.add(Dropout(.2))
model.add(ELU())

# Fully connected layer 1
model.add(Dense(512))
model.add(Dropout(.5))
model.add(ELU())

# Fully connected layer 2
model.add(Dense(50))
model.add(ELU())

model.add(Dense(1))

adam = Adam(lr=0.0001)

model.compile(optimizer=adam, loss="mse", metrics=['accuracy'])

print("Model summary:\n", model.summary())


## 4. Train model
batch_size = 32
nb_epoch = 20

# Save model weights after each epoch
checkpointer = ModelCheckpoint(filepath="./tmp/v2-weights.{epoch:02d}-{val_loss:.2f}.hdf5", verbose=1, save_best_only=False)

# Train model using generator
model.fit_generator(train_generator, 
                    samples_per_epoch=len(train_samples), 
                    validation_data=validation_generator,
                    nb_val_samples=len(validation_samples), nb_epoch=nb_epoch,
                    callbacks=[checkpointer])


## 5. Save model

model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)
    
model.save_weights("model.h5")
print("Saved model to disk")