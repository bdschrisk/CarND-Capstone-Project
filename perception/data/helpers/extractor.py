import os
import sys
import yaml
import argparse

import logging

from datetime import datetime
from PIL import Image


def ticks(dt):
    return int((dt - datetime(1, 1, 1)).total_seconds() * 10000000)

# Extract dataset from Bosch full size images
def extract_images_labels(input_yaml, output_path):
    images = yaml.load(open(input_yaml, 'rb').read())

    for i in range(len(images)):
        path = os.path.abspath(os.path.join(os.path.dirname(input_yaml), images[i]['path']))
        
        if (not os.path.exists(path)):
            continue
        
        image_raw = Image.open(path)
        img_w, img_h = image_raw.size

        b = 0
        for box in images[i]['boxes']:
            # extract portion and save image and label
            if box['x_max'] < box['x_min']:
                box['x_max'], box['x_min'] = box['x_min'], box['x_max']
            if box['y_max'] < box['y_min']:
                box['y_max'], box['y_min'] = box['y_min'], box['y_max']

            width = box['x_max'] - box['x_min']
            height = box['y_max'] - box['y_min']

            x_min = max(0, box['x_min'])
            y_min = max(0, box['y_min'])
            x_max = min(box['x_max'], img_w)
            y_max = min(box['y_max'], img_h)

            try:
                if width < 0 or height < 0:
                    logging.warning('Box smaller than 1 ({} {} {} {}) from {}'.format(x_min, y_min, x_max, y_max, path))
                else:
                    label = box['label']
                    image_crop = image_raw.crop((x_min, y_min, x_max, y_max))
                    new_file = "{}-{}.png".format(ticks(datetime.utcnow()), b)
                    path = os.path.join(output_path, label.lower())
                    # write images and label
                    if (not os.path.exists(path)):
                        os.mkdir(path)

                    image_crop.save(os.path.join(path, new_file), "PNG")
            except:
                logging.warning('Failed to extract box ({} {} {} {}) from {}'.format(x_min, y_min, x_max, y_max, path))

            b += 1

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'yaml_path',
        type=str,
        help='Path to the yaml file'
    )

    parser.add_argument(
        'output_path',
        type=str,
        nargs='?',
        default='',
        help='Path to output image folder. This is where the images from the run will be saved.'
    )

    args = parser.parse_args()

    print("Constructing datasets...")

    if (not os.path.exists(args.output_path)):
        os.mkdir(args.output_path)

    extract_images_labels(args.yaml_path, args.output_path)

    print("Done!")
