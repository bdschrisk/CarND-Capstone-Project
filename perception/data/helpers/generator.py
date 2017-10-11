import numpy as np
import matplotlib.image as mpimg
import cv2
from PIL import Image
import time
import os
import random
import logging
import keras.utils as utils
import imghdr
from math import *

from model import modelutils as mut

logging.basicConfig(filename='./data/training.log', level=logging.DEBUG)

# Gets a uniform random from the interval 0-1
def random_uniform(min = 0.0, max = 1.0):
    return min + (np.random.uniform() * (max - min))

# Noise function
def add_noise(x, noise = 0.2, channel = 2):
    if (noise == 0):
        return x

    rand = (0.5 + random_uniform(0., noise))
    x[:,:,channel] = x[:,:,channel] * rand
    x = np.clip(x, 0, 255).astype(np.uint8)

    return x

# Adds noise to an image
def add_image_noise(img, noise):
    aug_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    aug_image = add_noise(aug_image, noise)
    aug_image = cv2.cvtColor(aug_image, cv2.COLOR_HSV2RGB)

    return aug_image

def sample(X, y, sample_size):
    args = np.random.choice(len(X), sample_size, replace=False)
    return np.array(X)[args], np.array(y)[args]

def pad(image, new_size):
    if (image.width == new_size[0] and image.height == new_size[1]):
        return image

    left = int(max(floor((new_size[0] - image.width) / 2.0), 0))
    top = int(max(floor((new_size[1] - image.height) / 2.0), 0))

    new = Image.new("RGB", new_size, color = "black")
    new.paste(image, box = (left, top))

    return new

def generator(path, label_dict, batch_size = 32, noise = 0.3, flip = True):    
    image_types = ("png", "jpg", "jpeg", "bmp")
    nb_classes = len(label_dict)
    class_labels = list(set(label_dict.values()))

    # enumerate paths and labels into list
    images = []
    labels = []

    for dir in os.listdir(path):
        sub_path = os.path.join(path, dir)
        if (os.path.isdir(sub_path) and dir in label_dict):
            for img_name in os.listdir(sub_path):
                img_path = os.path.join(sub_path, img_name)

                if (imghdr.what(img_path) in image_types):
                    images.append(img_path)
                    labels.append(label_dict[dir])

    errors = 0
    num_samples = len(labels)
    if (num_samples == 0):
        print("Found {}".format(num_samples))

    # do generator
    while 1:
        for offset in range(0, num_samples, batch_size):
            if (offset == 0):
                images, labels = sample(images, labels, num_samples)

            batch_x, batch_y = images[offset:offset+batch_size], labels[offset:offset+batch_size]

            X = []
            y = []

            max_width = 0
            max_height = 0

            for i in range(len(batch_y)):
                try:
                    image = Image.open(batch_x[i])
                    label = batch_y[i]

                    max_width = max(max_width, image.width)
                    max_height = max(max_height, image.height)

                    X.append(image)
                    y.append(label)
                except Exception as ex:
                    logging.warning("-- Error occurred while processing file '{}'\n - {}".format(batch_x[0], ex))

            if (len(X) < batch_size and len(X) > 0): 
                logging.warning("-- Repeating images as not enough data.\n- Images: [{}]".format(["'{}',".format(b) for b in batch_x])) 
                while (len(X) < batch_size):
                    idx = np.random.randint(0, len(X))
                    image = X[idx]
                    label = y[idx]
                    X.append(image)
                    y.append(label)
            
            X_batch = []
            y_batch = []

            for i in range(len(batch_y)):
                img = pad(X[i], (max_width, max_height))

                img = np.array(img)
                lbl = y[i]

                rand = np.random.uniform()
                if (rand >= (1.0 - noise)):
                    img = add_image_noise(img, noise)
                
                X_batch.append(img)
                y_batch.append(lbl)


            X_batch = np.array(X_batch).astype(np.float32)
            y_batch = mut.one_hot_encode(y_batch, class_labels)

            yield (X_batch, y_batch)