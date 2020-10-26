#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

class ImagePublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ImagePublisherNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # read camera
        self.cap = cv.VideoCapture(2)
        # bridge between opencv and ros
        self.bridge = CvBridge()
        # construct publisher
        self.pub = rospy.Publisher('~duckie_cam/compressed', CompressedImage, queue_size=10)

        # TODO This should be a param but set to fixed for now
        self.cam_rate = 10

    def run(self):
        rate = rospy.Rate(self.cam_rate) 
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
                self.pub.publish(img_msg)
            else:
                continue
            rate.sleep()

    def on_shutdown(self):
        self.cap.release()

if __name__ == '__main__':
    # create the node
    node = ImagePublisherNode(node_name='image_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()