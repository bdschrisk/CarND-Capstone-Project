# Adapted from: https://github.com/tdeboissiere/DeepLearningImplementations/blob/master/DenseNet/densenet.py

from keras.models import Model
from keras.layers.core import Dense, Dropout, Activation
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import AveragePooling2D
from keras.layers.pooling import GlobalAveragePooling2D
from keras.layers import Input, concatenate
from keras.layers.normalization import BatchNormalization
from keras import regularizers
import keras.backend as K

from keras.models import model_from_json

from model.extensions.layers.resize import Resize
from model.extensions.layers.scale import Scale
from model.extensions.layers.normalizer import Normalizer
from model import modelconfig as mf

import os
import glob

K.set_image_dim_ordering('tf')

class DenseNet(object):

    def composite(self, x, nb_filter, dropout_rate=None, weight_decay=1E-4):
        """Apply BatchNorm, Relu 3x3Conv2D, optional Dropout
    
        :param x: Input keras network
        :param nb_filter: int -- number of filters
        :param dropout_rate: int -- dropout rate
        :param weight_decay: int -- weight decay factor

        :returns: keras network with b_norm, relu and convolution2d added
    
        :rtype: keras network
    
        """

        x = BatchNormalization(gamma_regularizer=regularizers.l2(weight_decay),
                               beta_regularizer=regularizers.l2(weight_decay))(x)

        x = Activation('relu')(x)

        x = Convolution2D(nb_filter, (3, 3),
                          kernel_initializer="he_uniform",
                          padding="same",
                          use_bias=False,
                          kernel_regularizer=regularizers.l2(weight_decay))(x)

        if dropout_rate:
            x = Dropout(dropout_rate)(x)

        return x

    def transition(self, x, nb_filter, dropout_rate=None, weight_decay=1E-4):
        """Apply BatchNorm, Relu 1x1Conv2D, optional Dropout and Maxpooling2D
    
        :param x: keras model
        :param nb_filter: int -- number of filters
        :param dropout_rate: int -- dropout rate
        :param weight_decay: int -- weight decay factor

        :returns: model

        :rtype: keras model, after applying batch_norm, relu-conv, dropout, maxpool

        """

        x = BatchNormalization(gamma_regularizer=regularizers.l2(weight_decay),
                               beta_regularizer=regularizers.l2(weight_decay))(x)

        x = Activation('relu')(x)

        x = Convolution2D(nb_filter, (1, 1),
                          kernel_initializer="he_uniform",
                          padding="same",
                          use_bias=False,
                          kernel_regularizer=regularizers.l2(weight_decay))(x)

        if dropout_rate:
            x = Dropout(dropout_rate)(x)
    
        x = AveragePooling2D((2, 2), strides=(2, 2))(x)

        return x
    
    def denseblock(self, x, nb_layers, nb_filter, growth_rate, dropout_rate=None, weight_decay=1E-4):
        """ Build a denseblock where the output of each
           composite is fed to subsequent ones

        :param x: keras model
        :param nb_layers: int -- the number of layers of conv_
                          factory to append to the model.
        :param nb_filter: int -- number of filters
        :param dropout_rate: int -- dropout rate
        :param weight_decay: int -- weight decay factor

        :returns: keras model with nb_layers of composite appended

        :rtype: keras model

        """

        list_feat = [x]

        if K.image_dim_ordering() == "th":
            concat_axis = 1
        elif K.image_dim_ordering() == "tf":
            concat_axis = -1

        for i in range(nb_layers):
            x = self.composite(x, growth_rate, dropout_rate, weight_decay)
            list_feat.append(x)
            x = concatenate(list_feat, axis=concat_axis)
            nb_filter += growth_rate

        return x, nb_filter


    def __init__(self, nb_classes, img_dim, depth, nb_dense_block, growth_rate, nb_filter, dropout_rate=None, weight_decay=1E-4):

        """ Build the DenseNet model

        :param nb_classes: int -- number of classes
        :param img_dim: tuple -- (channels, rows, columns)
        :param depth: int -- how many layers
        :param nb_dense_block: int -- number of dense blocks to add to end
        :param growth_rate: int -- number of filters to add
        :param nb_filter: int -- number of filters
        :param dropout_rate: float -- dropout rate
        :param weight_decay: float -- weight decay

        :returns: keras model with nb_layers of composite appended

        :rtype: keras model

        """

        model_input = Input(shape=img_dim)
        #model_input = Resize(size = (24, 50), axis = (1, 2))(model_input)
        # perform scaling as per paper using a learning approach
        x = Normalizer()(model_input)

        assert (depth - 4) % 3 == 0, "Depth must be 3 N + 4"

        # layers in each dense block

        nb_layers = int((depth - 4) / 3)

        # Initial convolution

        x = Convolution2D(nb_filter, (3, 3),
                          kernel_initializer="he_uniform",
                          padding="same",
                          name="initial_conv2D",
                          use_bias=False,
                          kernel_regularizer=regularizers.l2(weight_decay))(x)

        # Add dense blocks
        for block_idx in range(nb_dense_block - 1):
            x, nb_filter = self.denseblock(x, nb_layers, nb_filter, growth_rate,
                                      dropout_rate=dropout_rate,
                                      weight_decay=weight_decay)

            # add transition
            x = self.transition(x, nb_filter, dropout_rate=dropout_rate,
                           weight_decay=weight_decay)

        # The last denseblock does not have a transition
        x, nb_filter = self.denseblock(x, nb_layers, nb_filter, growth_rate,
                                  dropout_rate=dropout_rate,
                                  weight_decay=weight_decay)

        x = BatchNormalization(gamma_regularizer=regularizers.l2(weight_decay),
                               beta_regularizer=regularizers.l2(weight_decay))(x)

        x = Activation('relu')(x)

        x = GlobalAveragePooling2D()(x)

        x = Dense(nb_classes,
                  activation='softmax',
                  kernel_regularizer=regularizers.l2(weight_decay),
                  bias_regularizer=regularizers.l2(weight_decay))(x)

        densenet = Model(name="DenseNet", inputs=model_input, outputs=x)

        self.model = densenet
    
    def predict(self, sample):
        return self.model.predict(sample, batch_size=1)

    def load(self, model = None):
        self.model = (model or DenseNet.default())

    @staticmethod
    def default():
        """ Loads a default DenseNet model """

        model = DenseNet(mf.output_size, mf.image_size, mf.depth, mf.dense_blocks, mf.growth_rate, 
                        mf.filters, mf.dropout, mf.weight_decay)

        return model
    

    def save_model(self, path):
        """ Saves the model graph to the specified path """

        model_json = self.model.to_json()
        
        with open(os.path.join(mf.model_path, mf.model_weights_name), mode = "w") as f:
            f.write(model_json)

    @staticmethod
    def load_model(path):
        """ Loads the model graph from the specified path """

        model_json = None
        with open(path, mode = "r") as f:
            model_json = f.read()

        model = model_from_json(model_json, custom_objects={ 'Resize' : Resize, 'Scale': Scale })
        model = DenseNet(model)

        return model

    def save_weights(self, path = None):
        """ Saves the model weights to the specified weights file, or default """

        path = (path or os.path.join(mf.model_path, mf.model_weights_name))

        self.model.save_weights(path)

    def load_weights(self, path):
        """ Loads the model weights from the specified weights file, or default """

        path = (path or os.path.join(mf.model_path, mf.model_weights_name))

        self.model.load_weights(path)


