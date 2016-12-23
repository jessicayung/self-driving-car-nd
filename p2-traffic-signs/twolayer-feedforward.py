"""
Two-layer feedforward network to classify traffic signs

Author: Jessica Yung
December 2016
"""

# Import packages
import numpy as np
import tensorflow as tf
import time
import pickle
from sklearn.utils import shuffle

"""
Reference: https://www.tensorflow.org/versions/r0.11/tutorials/mnist/pros/index.html#deep-mnist-for-experts
"""
### IMPORT AND PREPROCESS DATA ###

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

# Normalise input (images still in colour)
X_train_norm = (X_train - X_train.mean()) / (np.max(X_train) - np.min(X_train))
X_test_norm = (X_test - X_test.mean()) / (np.max(X_test) - np.min(X_test))

X_train = X_train_norm
X_test = X_test_norm

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

# Network parameters
n_fc1 = 512
n_fc2 = 128
n_input = 32*32*3

# Model parameters
learning_rate = 0.001
initial_learning_rate = learning_rate
training_epochs = 15
batch_size = 100
display_step = 1
dropout = 0.75
anneal_mod_frequency = 15
annealing_rate = 1

print_accuracy_mod_frequency = 1

# tf Graph input
x_unflattened = tf.placeholder("float", [None, 32, 32, 3])
x = tf.reshape(x_unflattened, [-1, n_input])

y_rawlabels = tf.placeholder("int32", [None])
y = tf.one_hot(y_rawlabels, depth=43, on_value=1., off_value=0., axis=-1)

## Create model

def two_feedforward_network(model_x, model_weights, model_biases, model_dropout):
    # Fully connected layer 1
    fc1 = tf.add(tf.matmul(model_x, model_weights['fc1']), model_biases['fc1'])
    fc1 = tf.nn.relu(fc1)
    fc1 = tf.nn.dropout(fc1, model_dropout)
    # Fully connected layer 2
    fc2 = tf.add(tf.matmul(fc1, model_weights['fc2']), model_biases['fc2'])
    fc2 = tf.nn.relu(fc2)
    fc2 = tf.nn.dropout(fc2, model_dropout)
    # Output layer
    output = tf.add(tf.matmul(fc2, model_weights['out']), model_biases['out'])
    # Note: Softmax is outside the model
    return output


## Store layers weight & bias

# NEW: initialise neurons with slightly positive initial bias
# to avoid dead neurons.
def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    # alt: tf.random_normal(shape)
    return tf.Variable(initial)

def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)

weights = {
    'fc1': weight_variable([n_input, n_fc1]),
    'fc2': weight_variable([n_fc1, n_fc2]),
    'out': weight_variable([n_fc2, n_classes])
}

biases = {
    'fc1': bias_variable([n_fc1]),
    'fc2': bias_variable([n_fc2]),
    'out': bias_variable([n_classes])
}

# Construct model
pred = two_feedforward_network(x, weights, biases, dropout)


# Define loss and optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(pred, y))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)


# Function to initialise the variables
init = tf.initialize_all_variables()

### RUN MODEL ###
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
            batch_x, batch_y = np.array(X_train[i*batch_size:(i+1)*batch_size]), \
                               np.array(y_train[i*batch_size:(i+1)*batch_size])
            # tf.train.batch([X_train, y_train], batch_size=100, enqueue_many=True)
            # Run optimization op (backprop) and cost op (to get loss value)
            _, c = sess.run([optimizer, cost], feed_dict={x_unflattened: batch_x, y_rawlabels: batch_y})
            # Compute average loss
            avg_cost += c / total_batch
            # print(avg_cost)
        # Display logs per epoch step
        if epoch % display_step == 0:
            print("Epoch:", '%04d' % (epoch+1), "cost=",
                  "{:.9f}".format(avg_cost))
            last_epoch_time = epoch_time
            epoch_time = time.time()
            print("Time since start: ", epoch_time - init_time)
            print("Time since last epoch: ", epoch_time - last_epoch_time)
        # Anneal learning rate
        if (epoch + 1) % anneal_mod_frequency == 0:
            learning_rate *= annealing_rate
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
    # print("Time to calculate accuracy on training set: ", train_predict_time - epoch_time)
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
