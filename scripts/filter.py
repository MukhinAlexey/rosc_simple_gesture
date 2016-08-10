#!/usr/bin/python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os


# noinspection PyPep8Naming
class Filter:
    def __init__(self):
        self.node_name = "stratom_image_filter"
        rospy.init_node(self.node_name)

        rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()

        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.img_pub = rospy.Publisher('filtered_image', Image, queue_size=10)
        rospy.loginfo("Loading Filter Node...")

    def depth_callback(self, ros_image):
        try:
            inImg = self.bridge.imgmsg_to_cv2(ros_image)
        except CvBridgeError, e:
            print e

        # noinspection PyUnboundLocalVariable
        inImgarr = np.array(inImg, dtype=np.uint16)

        outImg = self.process_depth_image(inImgarr)

        sys.stdout = open(os.devnull, "w")
        imgmsg = self.bridge.cv2_to_imgmsg(outImg, "8UC1")
        sys.stdout = sys.__stdout__

        self.img_pub.publish(imgmsg)

    @staticmethod
    def process_depth_image(inImg):

        np.clip(inImg, 0, 1028, inImg)
        inImg >>= 2
        inImg = inImg.astype(np.uint8)
        ret, inImg = cv2.threshold(inImg, 70, 255, cv2.THRESH_BINARY)

        height, width = inImg.shape[:2]
        crop_Img = inImg[height * 1 / 4:height * 3 / 4, width * 1 / 4:width * 3 / 4]

        # cv2.imshow("Cropped Image",crop_Img)
        # cv2.waitKey(3)
        return crop_Img

    @staticmethod
    def cleanup():
        print "Shutting down filter node."
        cv2.destroyAllWindows()


# noinspection PyUnusedLocal
def main(args):
    try:
        Filter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down filter node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)