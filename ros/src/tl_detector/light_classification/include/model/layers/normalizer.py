import keras.backend as K
from keras.layers.core import Layer

import numpy as np

class Normalizer(Layer):

    def __init__(self, range = (-1, 1.0), **kwargs):
        super(Normalizer, self).__init__(**kwargs)
        self.range = range

    def call(self, x, mask=None):
        return ((x - K.min(x)) / (255 - K.min(x))) * (self.range[1] - self.range[0]) + self.range[0]
    
    def get_output_shape_for(self, input_shape):
        return input_shape
    
    def get_config(self):
        config = {'range': self.range}
        base_config = super(Normalizer, self).get_config()
        return dict(list(base_config.items()) + list(config.items()))