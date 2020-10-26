#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

class ColorDetectorNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ColorDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.Perception)
        # bridge between opencv and ros
        self.bridge = CvBridge()
        # construct subscriber for images
        self.sub = rospy.Subscriber('~duckie_cam/compressed', CompressedImage, self.callback)
        # construct publisher for debug images
        self.pub = rospy.Publisher('~debug_images/compressed', CompressedImage, queue_size=10)

        # TODO this should be a param but fixed for now
        self.color = 'yellow'

    def callback(self, data):
        # convert ros compressed img to opencv
        # pass to color detector
        # pass to bounding box drawing

        # convert back to ros compressed image
        # publish
        pass

    # returns the location of bounding boxes
    def detectColors(self, img):
        img = self.whiteBalance(img)
        img = cv.GaussianBlur(img,(5,5),cv.BORDER_DEFAULT)

        img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        lower, upper = self.colorToHSVRange(self.color)

        isolated_color = cv.inRange(img, lower, upper)

        pass

    @staticmethod
    def whiteBalance(img):
        result = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        avg_a = np.average(result[:, :, 1])
        avg_b = np.average(result[:, :, 2])
        result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result = cv.cvtColor(result, cv.COLOR_LAB2BGR)
        return result

    @staticmethod
    def boundingBoxColors(img, bounding_boxes):
        pass

    @staticmethod
    def colorToHSVRange(color):
        if color == 'yellow':
            return np.array([22, 93, 0], dtype="uint8"), np.array([45, 255, 255], dtype="uint8")
        else:
            raise Exception('undefined color')

if __name__ == '__main__':
    # create the node
    node = ColorDetectorNode(node_name='color_detector_node')
    # keep spinning
    rospy.spin()