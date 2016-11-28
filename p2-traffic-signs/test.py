

# Import packages
import numpy as np`
import tensorflow as tf
import time
import pickle
from sklearn.utils import shuffle
# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
# import cv2
# import os

"""
Reference: https://www.tensorflow.org/versions/r0.11/tutorials/mnist/pros/index.html#deep-mnist-for-experts
"""

f = open('trafficsign', 'w')

# Import and read training and test data
training_file = "traffic-sign-data/train.p"
testing_file = "traffic-sign-data/test.p"

with open(training_file, mode='rb') as f:
    train = pickle.load(f)
with open(testing_file, mode='rb') as f:
    test = pickle.load(f)
    
X_train, y_train = train['features'], train['labels']
X_test, y_test = test['features'], test['labels']


# Shuffle training examples
X_train, y_train = shuffle(X_train, y_train)


# Data summary to test Python output
n_train = len(X_train)
n_test = len(X_test)
image_shape = X_train[0].shape
n_classes = len(set(y_train))

print("Number of training examples =", n_train)
print("Number of testing examples =", n_test)
print("Image data shape =", image_shape)
print("Number of classes =", n_classes)

### MODEL ###
# Model parameters
learning_rate = 0.01
initial_learning_rate = learning_rate
training_epochs = 2
batch_size = 50
display_step = 1
dropout = 0.75
anneal_mod_frequency = 15
annealing_rate = 0.9

print_accuracy_mod_frequency = 5

# Additional parameters for multilayer perceptron
# n_hidden_1 = 256 # 1st layer number of features
# n_hidden_2 = 256 # 2nd layer number of features
# n_input = 3072 # Data input (img shape: 32*32*3)


# tf Graph input
x_unflattened = tf.placeholder("float", [None, 32, 32, 3])
x = x_unflattened
# TODO: Check this, should have one more dim?
y_rawlabels = tf.placeholder("int32", [None])
y = tf.one_hot(y_rawlabels, depth=43, on_value = 1., off_value = 0., axis=-1)


# Convnet helper functions
def conv2d(x, W, b, strides=1):
    # Conv2D wrapper, with bias and relu activation
    # strides = [batch, in_height, in_width, channels]
    x = tf.nn.conv2d(x, W, strides=[1, strides, strides, 1], padding='SAME')
    x = tf.nn.bias_add(x, b)
    return tf.nn.relu(x)


def maxpool2d(x, k=2):
    # MaxPool2D wrapper
    return tf.nn.max_pool(x, ksize=[1, k, k, 1], strides=[1, k, k, 1],
                          padding='SAME')


# Create model
def conv_net(x, weights, biases, dropout):
    # Convolution Layer
    conv1 = conv2d(x, weights['wc1'], biases['bc1'])
    # Max Pooling (down-sampling)
    conv1 = maxpool2d(conv1, k=2)

    # Convolution Layer
    conv2 = conv2d(conv1, weights['wc2'], biases['bc2'])
    # Max Pooling (down-sampling)
    conv2 = maxpool2d(conv2, k=2)

    # Fully connected layer
    # Reshape conv2 output to fit fully connected layer input
    fc1 = tf.reshape(conv2, [-1, weights['wd1'].get_shape().as_list()[0]])
    fc1 = tf.add(tf.matmul(fc1, weights['wd1']), biases['bd1'])
    fc1 = tf.nn.relu(fc1)
    # Apply Dropout
    fc1 = tf.nn.dropout(fc1, dropout)

    # Output, class prediction
    out = tf.add(tf.matmul(fc1, weights['out']), biases['out'])
    return out


# Store layers weight & bias

# NEW: initiallise neurons with slightly positive initial bias 
# to avoid dead neurons.
def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    # alt: tf.random_normal(shape)
    return tf.Variable(initial)

def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)


weights = {
    # 5x5 conv, 1 input, 32 outputs
    # CHANGE IF SWITCH TO GRAYSCALE OR COLOR
    'wc1': weight_variable([5, 5, 3, 32]),
    # 5x5 conv, 32 inputs, 64 outputs
    'wc2': weight_variable([5, 5, 32, 64]),
    # fully connected, 7*7*64 inputs, 1024 outputs
    'wd1': weight_variable([4096, 1024]),
    # 1024 inputs, 10 outputs (class prediction)
    'out': weight_variable([1024, n_classes])
}

biases = {
    'bc1': bias_variable([32]),
    'bc2': bias_variable([64]),
    'bd1': bias_variable([1024]),
    'out': bias_variable([n_classes])
}


# Construct model
pred = conv_net(x, weights, biases, dropout)


# Define loss and optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(pred, y))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)


# Initializing the variables
init = tf.initialize_all_variables()


# Launch the graph
with tf.Session() as sess:
    sess.run(init)
    
    # Initialise time logs
    init_time = time.time()
    epoch_time = init_time

    # Training cycle
    for epoch in range(training_epochs):
        avg_cost = 0.
        
        total_batch = int(n_train/batch_size)
        # Loop over all batches
        for i in range(total_batch):
            batch_x, batch_y = np.array(X_train[i*batch_size:(i+1)*batch_size]), np.array(y_train[i*batch_size:(i+1)*batch_size])
            # tf.train.batch([X_train, y_train], batch_size=100, enqueue_many=True)
            # Run optimization op (backprop) and cost op (to get loss value)
            _, c = sess.run([optimizer, cost], feed_dict={x_unflattened: batch_x, y_rawlabels: batch_y})
            # Compute average loss
            avg_cost += c / total_batch
            # print(avg_cost)
        # Display logs per epoch step
        if epoch % display_step == 0:
            print("Epoch:", '%04d' % (epoch+1), "cost=", \
                "{:.9f}".format(avg_cost))
            last_epoch_time = epoch_time
            epoch_time = time.time()
            print("Time since start: ", epoch_time - init_time)
            print("Time since last epoch: ", epoch_time - last_epoch_time)
        # Anneal learning rate
        print("Anneal learning rate every ", anneal_mod_frequency, " epochs by ", 1 - annealing_rate)
        if (epoch + 1) % anneal_mod_frequency == 0:
            learning_rate = learning_rate * annealing_rate
            print("New learning rate: ", learning_rate)
           
        if (epoch + 1) % print_accuracy_mod_frequency == 0:
            correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
            accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
            print("Accuracy (test):", accuracy.eval({x_unflattened: X_test, y_rawlabels: y_test}))
            
    print("Optimization Finished!")

    # Test model
    correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
    # Calculate accuracy
    # accuracy_train = tf.reduce_mean(tf.cast(correct_prediction, "float"))
    # print("Accuracy (train):", accuracy_train.eval({x_unflattened: X_train, y_rawlabels: y_train}))
    train_predict_time = time.time()
    #print("Time to calculate accuracy on training set: ", train_predict_time - epoch_time)
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
    print("Accuracy (test):", accuracy.eval({x_unflattened: X_test, y_rawlabels: y_test}))
    test_predict_time = time.time()
    print("Time to calculate accuracy on test set: ", test_predict_time - train_predict_time)
    
    # Print parameters for reference
    print("Parameters:")
    print("Learning rate (initial): ", initial_learning_rate)
    print("Anneal learning rate every ", anneal_mod_frequency, " epochs by ", 1 - annealing_rate)
    print("Learning rate (final): ", learning_rate)
    print("Training epochs: ", training_epochs)
    print("Batch size: ", batch_size)
    print("Dropout: ", dropout)

f.close()