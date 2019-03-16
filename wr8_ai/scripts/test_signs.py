#!/usr/bin/env python

import rospy
import json
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np

from wr8_ai.yolo import fps
import wr8_ai.detector_ncs as det

import time


class ImageReceiverROS:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera", Image, self.callback_img, queue_size=1)
        self.image_sub = rospy.Subscriber("camera_compr", CompressedImage, self.callback_img_compressed, queue_size=1)

        self.cv_image = None
        self.cv_image_comp = None

    def callback_img(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def callback_img_compressed(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        self.cv_image_comp = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def get_image(self):
        return self.cv_image

    def get_image_compressed(self):
        return self.cv_image_comp


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

    graph_path = rospy.get_param('~graph_path')
    config_path = rospy.get_param('~config_path')
    fps_msr = rospy.get_param('~fps_msr', True)

    fps_meter = fps.FPSMeter()

    rospy.loginfo('Start processing')

    detector = det.DetectorNCS()
    if not detector.init(0, graph_path, config_path):
        rospy.logerr('Failed to initialize detector')

    img_rcvr = ImageReceiverROS()
    img_pub = ImagePublisherROS()

    skip_cntr = 0

    while not rospy.is_shutdown():
        image = img_rcvr.get_image_compressed()

        if image is None:
            rospy.sleep(0.01)   # 10 ms
            skip_cntr += 1
            if skip_cntr > 300:
                rospy.logwarn('No image for 3 seconds...')
                skip_cntr = 0
            continue
        
        render_img = image.copy()

        start = time.time()

        boxes, box_img = detector.get_signs(cv_img=image, render_img=render_img)

        if fps_msr:
            fps_meter.update(time.time() - start)

            if fps_meter.milliseconds > 5000:
                fps_meter.print_statistics()
                fps_meter.reset()

        img_pub.publish(box_img)

    #     cv2.imshow('2', image)
    #     key = cv2.waitKey(10)
    #     if key == 27:
    #         break

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
