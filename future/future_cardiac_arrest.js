model = tf.sequential(); // Initialize a sequential model
model.add(tf.layers.conv2d({ // Add a 2D convolutional layer
    inputShape: [28, 28, 1], // The input shape is 28x28 pixels with 1 color channel (grayscale)
    kernelSize: 3,           // The size of the convolutional kernel is 3x3
    filters: 8,              // The layer has 8 filters (output depth is 8)
    activation: 'relu'       // Use ReLU activation function
}));
model.add(tf.layers.maxPooling2d({ // Add a 2D max pooling layer
    poolSize: [2, 2]               // The size of the pooling window is 2x2
}));
model.add(tf.layers.conv2d({ // Add another 2D convolutional layer
    filters: 16,              // This layer has 16 filters (output depth is 16)
    kernelSize: 3,            // The size of the convolutional kernel is 3x3
    activation: 'relu'        // Use ReLU activation function
}));
model.add(tf.layers.maxPooling2d({ // Add another 2D max pooling layer
    poolSize: [2, 2]               // The size of the pooling window is 2x2
}));
model.add(tf.layers.flatten()); // Flatten the 3D output to 1D for the fully connected layers
model.add(tf.layers.dense({ // Add a dense (fully connected) layer
    units: 128,            // The layer has 128 neurons
    activation: 'relu'     // Use ReLU activation function
}));
model.add(tf.layers.dense({ // Add the output dense (fully connected) layer
    units: 10,             // The layer has 10 neurons (one for each class in a 10-class classification problem)
    activation: 'softmax'  // Use softmax activation function to get probability distribution over the classes
}));
