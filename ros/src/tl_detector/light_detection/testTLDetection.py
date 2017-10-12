import unittest
from light_detection.tl_detection import TLDetection
import os
from PIL import Image
import glob

TEST_DIR=os.path.join(os.path.dirname(__file__),'test')

class Test_TLDetection(unittest.TestCase):
    def test_TLDetection(self):
        tld_detect = TLDetection()
        sim_image_path=  os.path.join(TEST_DIR,'image1.jpg')
        test_image=Image.open(sim_image_path)
        traffic_lights =  tld_detect.detect_traffic_lights(test_image)
        assert(len(traffic_lights)==3)

if __name__ == '__main__':
    unittest.main()
