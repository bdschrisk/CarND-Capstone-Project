import os
import rospy
import numpy as np

from styx_msgs.msg import TrafficLight
from include.model.KaNet import KaNet

class TLClassifier(object):
    def __init__(self):
        # load params
        self.classes = rospy.get_param("~tl_classes")
        self.values = rospy.get_param("~tl_values")
        self.weights_file = rospy.get_param("~tl_weights_file")

        self.model = KaNet(len(self.classes), (None, None, 3), 1.0, 0)
        self.model.load_weights(self.weights_file)
    
    def map_label(self, output):
        """ Maps the argmax of model output to the traffic light label """
        val = self.values[output]

        return np.uint8(val)
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """


        result = TrafficLight.UNKNOWN

        msize = image.shape[0]

        predictions = self.model.predict(image[:, :, :, 0:3], batch_size=msize)
        for i in range(predictions.shape[0]):
            pred_idx = np.argmax(predictions[i])
        #TODO Add total prob
        result = self.map_label(pred_idx)

        return result
