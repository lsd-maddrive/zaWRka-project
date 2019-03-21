#!/usr/bin/env python

import rospy
import json
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

from wr8_ai.yolo import fps
# import wr8_ai.detector_ncs as det

from wr8_ai.detectors import SignsDetector

import time

class ImagePublisherROS:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("netout/compressed", CompressedImage)

    def publish(self, cv_image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = np.array(cv2.imencode('.png', cv_image)[1]).tostring()

        self.image_pub.publish(msg)


def main():

    rospy.init_node('test_signs')

    sign_det = SignsDetector()
    if not sign_det.init():
        exit(1)

    fps_msr = rospy.get_param('~fps_msr', True)

    fps_meter = fps.FPSMeter()

    rospy.loginfo('Start processing')

    while not rospy.is_shutdown():
        start = time.time()

        current_signs_rbboxes = sign_det.get_signs()

        if fps_msr:
            fps_meter.update(time.time() - start)

            if fps_meter.milliseconds > 5000:
                fps_meter.print_statistics()
                fps_meter.reset()




if __name__ == '__main__':
    main()
