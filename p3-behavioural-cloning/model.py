## Imports
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
%matplotlib inline

from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.layers import Convolution2D, MaxPooling2D
from keras.optimizers import SGD, Adam, RMSprop
from keras.utils import np_utils
from keras.models import model_from_json

from sklearn.model_selection import train_test_split


## Import data
# Added header row manually to CSV.
driving_csv = pd.read_csv("data/driving_log.csv")

# Examine data
print("Number of datapoints: %d" % len(driving_csv))

driving_csv.head()

# Extract centre image and steering angle from table
# Format: X_path: centre image name, y: steeringa angle
X_path = [driving_csv.loc[i]["Centre Image"] \
              for i in range(len(driving_csv))]
y = [driving_csv.loc[i][" Steering Angle"] \
              for i in range(len(driving_csv))]

# Import images
X_images = [mpimg.imread(image_path) for image_path in X_path]

# View image
print("Images: %d" % len(X_images))
print("Sample image")
plt.imshow(X_images[0])
# X_images[0]

# Check image shape
print("Image shape: ", X_images[0].shape)


## Train-test split
X_train, X_test, y_train, y_test = train_test_split(X_images, y, test_size=0.1, random_state=42)

X_train = np.array(X_train)
X_test = np.array(X_test)
y_train = np.array(y_train)
y_test = np.array(y_test)


## Build model
model = Sequential()
model.add(Convolution2D(160, 3, 3, border_mode='same',
                        input_shape=(160,320,3)))
model.add(Activation('relu'))
model.add(Convolution2D(32, 3, 3))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))

model.add(Convolution2D(64, 3, 3, border_mode='same'))
model.add(Activation('relu'))
model.add(Convolution2D(64, 3, 3))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))

model.add(Flatten())
model.add(Dense(512))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(1))

# Compile model
sgd = SGD(lr=0.01, decay=1e-6, momentum=0.9, nesterov=True)
model.compile(loss='mean_squared_error',
              optimizer=sgd,
              metrics=['accuracy'])


## Train model
batch_size = 50
nb_epoch = 10

model.fit(X_train, y_train,
              batch_size=batch_size,
              nb_epoch=nb_epoch,
              validation_split=0.1,
              # validation_data=(X_test, y_test),
              show_accuracy=True,
              shuffle=True)


## Extract model data
# Save weights
model.save_weights("model.h5")

# Save model config (architecture)
json_string = model.to_json()
with open("model.json", "w") as f:
    f.write(json_string)    