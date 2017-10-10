import tensorflow as tf
import numpy as np
from PIL import Image
import os
import time
import tarfile
MODELS_DIR=os.path.join(os.path.dirname(__file__),'include')


def load_graph(graph_file):
    """Loads a frozen inference graph"""
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        image_tensor = graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        detection_classes = graph.get_tensor_by_name('detection_classes:0')
    return graph, image_tensor, detection_boxes,detection_scores,detection_classes

class TLDetection(object):
    def __init__(self):
        # load params
        self.model_path= os.path.join(MODELS_DIR,'ssd_mobilenet_v1_coco_11_06_2017','frozen_inference_graph.pb')
        self.detection_graph,self.image_tensor, self.detection_boxes,self.detection_scores,self.detection_classes = load_graph(self.model_path)

    def to_image_coords(self,boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
    
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
    
        return box_coords
        
    def filter_boxes(self,min_score, boxes, scores, classes, categories):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if classes[i] in categories and scores[i] >= min_score:
                idxs.append(i)
    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    
    def detect_boxes(self,
               image_tensor, 
               detection_boxes,
               detection_scores,
               detection_classes,
               image_np,
               runs=1):
  
        with tf.Session(graph=self.detection_graph) as sess:                
            # Actual detection.
        
            times = np.zeros(runs)
            for i in range(runs):
                t0 = time.time()
                (boxes, scores, classes) = sess.run([detection_boxes, detection_scores, detection_classes], 
                                                    feed_dict={image_tensor: image_np})
                t1 = time.time()
                times[i] = (t1 - t0) * 1000

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)

        return boxes, scores, classes, times

        
    def detect_traffic_lights(self,image):
        width = image.size[0]
        height = image.size[1]
        cropped_image = image.crop(
            (
                0,
                100,
                width,
                height-150
            )
        )
        image_np = np.expand_dims(np.asarray(cropped_image, dtype=np.uint8), 0)
        boxes,scores,classes,_ = self.detect_boxes(self.image_tensor,
                                      self.detection_boxes,
                                      self.detection_scores,
                                      self.detection_classes,
                                      image_np)
        confidence_cutoff = 0.2
        traffic_lights_class_id=10
        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores,classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes,[traffic_lights_class_id])
        width, height = cropped_image.size
        box_coords = self.to_image_coords(boxes, height, width)
        cropped_images = []
        print(box_coords)
        for box in box_coords:
            top,left,bottom,right = box
            traffic_light = cropped_image.crop(
                (
                   left,
                    top,
                    right,
                    bottom
                )
            )
            cropped_images.append(traffic_light)
        return cropped_images
