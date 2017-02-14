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

# Choose whether or not to use reduced dataset. 
# Reduced dataset halves the number of examples with steering angle = 0.
reduced = False

if reduced == True:
    csv_filepath = 'data-udacity/driving_log_reduced.csv'
else:
    csv_filepath = 'data-udacity/driving_log.csv'


# Save filepaths of images to `samples` to load into generator    
samples = []
with open(csv_filepath) as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)
# Remove header
samples = samples[1:]
 
# Split samples into training and validation sets to reduce overfitting
train_samples, validation_samples = train_test_split(samples, test_size=0.1)

# Define generator function
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = './data-udacity-flipped/IMG/'+batch_sample[0].split('/')[-1]
                center_image = mpimg.imread(name)
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)

            X_train = np.array(images)
            y_train = np.array(angles)
            
            yield shuffle(X_train, y_train)

# Compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)


## 2. Data Preprocessing functions

def resize(image):
    import tensorflow as tf  # This import is required here otherwise the model cannot be loaded in drive.py
    return tf.image.resize_images(image, 66, 200)


## 3. Model (data preprocessing incorporated into model)

model = Sequential()

# Crop 50 pixels from the top of the image and 20 from the bottom
model.add(Cropping2D(cropping=((50, 20), (0, 0)),
                     dim_ordering='tf', # default
                     input_shape=(160, 320, 3)))

# Resize the data
model.add(Lambda(resize))

model.add(Lambda(lambda x: x[:,:,:,0:1]))

model.add(Lambda(lambda x: (x/127.5) - 1.))

model.add(Convolution2D(16, 8, 8, subsample=(4, 4), border_mode="same"))
model.add(ELU())

model.add(Convolution2D(32, 5, 5, subsample=(2, 2), border_mode="same"))
model.add(ELU())

model.add(Convolution2D(64, 5, 5, subsample=(2, 2), border_mode="same"))

model.add(Flatten())
model.add(Dropout(.2))
model.add(ELU())

model.add(Dense(512))
model.add(Dropout(.5))
model.add(ELU())

model.add(Dense(1))

adam = Adam(lr=0.0001)

model.compile(optimizer=adam, loss="mse", metrics=['accuracy'])


## 4. Train model
batch_size = 32
nb_epoch = 20

# Save model weights after each epoch
checkpointer = ModelCheckpoint(filepath="./tmp/comma-weights.{epoch:02d}-{val_loss:.2f}.hdf5", verbose=1, save_best_only=False)

# Train model using generator
model.fit_generator(train_generator, 
                    samples_per_epoch=len(train_samples), 
                    validation_data=validation_generator,
                    nb_val_samples=len(validation_samples), nb_epoch=nb_epoch,
                    callbacks=[checkpointer])


## 5. Save model

model_json = model.to_json()
with open("model-comma.json", "w") as json_file:
    json_file.write(model_json)
    
model.save_weights("model.h5")
print("Saved model to disk")