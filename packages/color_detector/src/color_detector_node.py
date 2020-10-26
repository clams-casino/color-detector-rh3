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
        super(ColorDetectorNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # bridge between opencv and ros
        self.bridge = CvBridge()
        # construct subscriber for images
        self.sub = rospy.Subscriber('/hobojones/image_publisher_node/duckie_cam/compressed', CompressedImage, self.callback)
        # construct publisher for debug images
        self.pub = rospy.Publisher('~debug_images/compressed', CompressedImage, queue_size=10)

        # TODO this should be a param but fixed for now
        self.color = 'yellow'

        # TODO this should be a param but fixed for now
        self.downscale = 0.6

    def callback(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")

        img = self.resize(img, self.downscale)
        color_detections = self.detectColors(img)
        self.drawBoundingBoxes(img, color_detections)
        img = self.resize(img, 1/self.downscale)

        debug_msg = self.bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
        self.pub.publish(debug_msg)

    def detectColors(self, img):
        img = cv.GaussianBlur(img,(3,3),cv.BORDER_DEFAULT)
        img = self.whiteBalance(img)
        
        img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower, upper = self.colorToHSVRange(self.color)
        isolated_color = cv.inRange(img, lower, upper)

        isolated_color = cv.erode(isolated_color, None, iterations=5)
        isolated_color = cv.dilate(isolated_color, None, iterations=6)

        return isolated_color

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
    def resize(img, scale):
        width = int(img.shape[1] * scale)
        height = int(img.shape[0] * scale)
        return cv.resize(img, (width, height), interpolation=cv.INTER_AREA)

    @staticmethod
    def drawBoundingBoxes(img, color_detections):
        cnts, _ = cv.findContours(color_detections, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            x,y,w,h = cv.boundingRect(cnt)
            if w<15 or h<15 or w*h < 250:
                continue
            else:
                cv.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

    @staticmethod
    def colorToHSVRange(color):
        if color == 'yellow':
            return np.array([25, 30, 0], dtype="uint8"), np.array([45, 255, 255], dtype="uint8")
        else:
            raise Exception('undefined color')

if __name__ == '__main__':
    # create the node
    node = ColorDetectorNode(node_name='color_detector_node')
    # keep spinning
    rospy.spin()