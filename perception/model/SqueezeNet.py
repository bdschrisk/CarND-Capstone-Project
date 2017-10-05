from keras.layers import Activation, Dropout, Convolution2D, MaxPooling2D, Flatten, Dense
from keras.models import Sequential
from keras.models import Model
from keras.layers import Input, Activation, concatenate
from keras.layers import Flatten, Dropout
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import AveragePooling2D
from keras.layers.pooling import GlobalAveragePooling2D
from keras import regularizers
from keras.optimizers import Adam
import keras.backend as K

from model.extensions.layers.normalizer import Normalizer


K.set_image_dim_ordering('tf')

def SqueezeNet(nb_classes, image_size, dropout_rate, weight_decay):
    """ Keras Implementation of SqueezeNet(arXiv 1602.07360)
    
    :param nb_classes: total number of final categories
    :param image_size: image size, not including the batch dimension
    :dropout_rate: dropout rate
    
    :returns: Keras model
    
    """
    
    input = Input(shape=image_size)
    normalize = Normalizer()(input)
    
    conv1 = Convolution2D(96, (7, 7), activation='relu', kernel_initializer='glorot_uniform',
    					  strides=(2, 2), padding='same', name='conv1')(normalize)
    
    maxpool1 = MaxPooling2D(pool_size=(3, 3), strides=(2, 2), name='maxpool1')(conv1)
    
    
    fire2_squeeze = Convolution2D(16, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					 padding='same', name='fire2_squeeze')(maxpool1)
    
    fire2_expand1 = Convolution2D(64, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire2_expand1')(fire2_squeeze)
    
    fire2_expand2 = Convolution2D(64, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire2_expand2')(fire2_squeeze)
    
    
    merge2 = concatenate([fire2_expand1, fire2_expand2], axis=-1)
    
    
    fire3_squeeze = Convolution2D(16, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire3_squeeze')(merge2)
    
    fire3_expand1 = Convolution2D(64, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire3_expand1')(fire3_squeeze)
    
    fire3_expand2 = Convolution2D(64, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire3_expand2')(fire3_squeeze)
    
    merge3 = concatenate([fire3_expand1, fire3_expand2], axis=-1)
    
    fire4_squeeze = Convolution2D(32, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire4_squeeze')(merge3)
    
    fire4_expand1 = Convolution2D(128, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire4_expand1')(fire4_squeeze)
    
    fire4_expand2 = Convolution2D(128, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire4_expand2')(fire4_squeeze)
    
    
    merge4 = concatenate([fire4_expand1, fire4_expand2], axis=-1)
    
    
    maxpool4 = MaxPooling2D(pool_size=(3, 3), strides=(2, 2), name='maxpool4')(merge4)
    
    fire5_squeeze = Convolution2D(32, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire5_squeeze')(maxpool4)
    
    fire5_expand1 = Convolution2D(128, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire5_expand1')(fire5_squeeze)
    
    fire5_expand2 = Convolution2D(128, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire5_expand2')(fire5_squeeze)
    
    
    merge5 = concatenate([fire5_expand1, fire5_expand2], axis=-1)
    
    
    fire6_squeeze = Convolution2D(48, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire6_squeeze')(merge5)
    
    fire6_expand1 = Convolution2D(192, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire6_expand1')(fire6_squeeze)
    
    fire6_expand2 = Convolution2D(192, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire6_expand2')(fire6_squeeze)
    
    
    merge6 = concatenate([fire6_expand1, fire6_expand2], axis=-1)
    
    
    fire7_squeeze = Convolution2D(48, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire7_squeeze')(merge6)
    
    fire7_expand1 = Convolution2D(192, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire7_expand1')(fire7_squeeze)
    
    fire7_expand2 = Convolution2D(192, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire7_expand2')(fire7_squeeze)
    
    
    merge7 = concatenate([fire7_expand1, fire7_expand2], axis=-1)
    
    
    fire8_squeeze = Convolution2D(64, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire8_squeeze')(merge7)
    
    fire8_expand1 = Convolution2D(256, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire8_expand1')(fire8_squeeze)
    
    fire8_expand2 = Convolution2D(256, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire8_expand2')(fire8_squeeze)
    
    
    merge8 = concatenate([fire8_expand1, fire8_expand2], axis=-1)
    
    
    maxpool8 = MaxPooling2D(pool_size=(3, 3), strides=(2, 2), name='maxpool8')(merge8)
    
    fire9_squeeze = Convolution2D(64, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire9_squeeze')(maxpool8)
    
    fire9_expand1 = Convolution2D(256, (1, 1), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire9_expand1')(fire9_squeeze)
    
    fire9_expand2 = Convolution2D(256, (3, 3), activation='relu', kernel_initializer='glorot_uniform',
    					padding='same', name='fire9_expand2')(fire9_squeeze)
    
    
    merge9 = concatenate([fire9_expand1, fire9_expand2], axis=-1)
    
    
    conv10 = Convolution2D(nb_classes, (1, 1), kernel_initializer='glorot_uniform',
    					padding='valid', name='conv10')(merge9)
    
    # The size should match the output of conv10
    avgpool10 = GlobalAveragePooling2D(name='avgpool10')(conv10)

    softmax = Activation('softmax')(avgpool10)
    
    return Model(inputs=input, outputs=softmax)