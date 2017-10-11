from keras.layers import Activation, Dropout, Convolution2D, MaxPooling2D, Flatten, Dense
from keras.models import Sequential
from keras.models import Model
from keras.layers import Input, Activation, concatenate
from keras.layers.advanced_activations import PReLU
from keras.layers import Flatten, Dropout
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import AveragePooling2D
from keras.layers.pooling import GlobalAveragePooling2D
from keras import regularizers
from keras.optimizers import Adam
import keras.backend as K

from model.extensions.layers.normalizer import Normalizer


K.set_image_dim_ordering('tf')

def DeepNet(nb_classes, image_size, dropout_rate, weight_decay):
    """ KaNet
    
    :param nb_classes: total number of final categories
    :param image_size: image size, not including the batch dimension
    :dropout_rate: dropout rate
    
    :returns: Keras model
    
    """
    
    input = Input(shape=image_size)
    output = Normalizer()(input)
    
    output = Convolution2D(64, (5, 5), padding='valid')(output)
    # downsampling layer (avg pool)
    output = AveragePooling2D((4, 4), strides=(2, 2))(output)
    output = Activation('relu')(output)
    
    # second convolutional layer
    output = Convolution2D(32, (3, 3), padding='valid')(output)
    # downsampling layer (avg pool)
    output = AveragePooling2D((2, 2), strides=(1, 1))(output)
    output = Activation('relu')(output)

    # third convolutional layer
    output = Convolution2D(16, (2, 2), padding='valid')(output)
    # downsampling layer (avg pool)
    output = AveragePooling2D((2, 2), strides=(1, 1))(output)
    output = Activation('relu')(output)
    
    # fourth convolutional layer
    output = Convolution2D(8, (2, 2), padding='valid')(output)
    # downsampling layer (avg pool)
    output = AveragePooling2D((2, 2), strides=(1, 1))(output)
    output = Activation('relu')(output)
    
    # spatial features layer
    output = AveragePooling2D((2, 2), strides=(2, 2))(output)
    output = GlobalAveragePooling2D()(output)
    
    # Hidden layer 1
    output = Dense(256, W_regularizer=regularizers.l2(weight_decay), b_regularizer=regularizers.l2(weight_decay))(output)
    output = PReLU()(output)
    output = Dropout(dropout_rate)(output)
    
    softmax = Dense(nb_classes,
                  activation='softmax',
                  kernel_regularizer=regularizers.l2(weight_decay),
                  bias_regularizer=regularizers.l2(weight_decay))(output)

    return Model(inputs=input, outputs=softmax)
