#!/usr/bin/env python

import rospy
from wr8_ai.utils import ImageReceiverROS, ImagePublisherROS
import math

from std_msgs.msg import Float32
from wr8_ai.srv import WhiteLineDetection, WhiteLineDetectionResponse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
import time


class WhiteLineDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image', Image, self.callback_img, queue_size=1)

        self.white_pub = ImagePublisherROS('wl/image_raw')
        self.skip_cntr = 0

        self.cv_image = None
        self.last_coords = None

        rospy.Service('wl/detect', WhiteLineDetection, self.service_handler)

    def callback_img(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def service_handler(self, req):
        resp = WhiteLineDetectionResponse()

        if self.last_coords is not None:
            resp.y_coord = Float32(self.last_coords)
        else:
            resp.y_coord = Float32(-1)

        return resp

    def update(self):
        self.last_coords = self.get_white_line_coord()

    def get_white_line_coord(self):
        img = self.cv_image

        if img is None:
            self.skip_cntr += 1
            if self.skip_cntr > 1000000:
                rospy.loginfo('[White]: Skip - no image')
                self.skip_cntr = 0
            return -1

        orig_img = img

        cropped_img = img[40:]
        img = cropped_img

        lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

        best_gray = lab[:, :, 0]

        ret, thresh = cv2.threshold(best_gray, 170, 255, cv2.THRESH_BINARY)

        edges = cv2.Canny(thresh, 100, 200, None, 3)

        minLineLength = 20
        maxLineGap = 20

        img_render = img.copy()

        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        mask_img = np.zeros_like(cropped_img, dtype=np.uint8)

        # Get mean colors inside contours
        for i in xrange(0, len(contours)):
            mask = np.zeros(best_gray.shape, np.uint8)
            cv2.drawContours(mask, contours, i, 255, -1)

            # contour_mean_clr = cv2.mean(img, mask)

            lines = cv2.HoughLines(edges, 1, np.pi / 180, 90, None, 0, 0)
            if lines is not None:
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv2.line(mask_img, pt1, pt2, 255, 1, cv2.LINE_AA)
                    cv2.line(img_render, pt1, pt2, (0, 255, 255), 2, cv2.LINE_AA)

        pixels = np.argwhere(mask_img == 255)
        if len(pixels) > 0:
            y_mean = np.mean(pixels[:, 0])
        else:
            y_mean = -1

        self.white_pub.publish( img_render )

        return y_mean

def main():
    rospy.init_node('wl_det')

    det = WhiteLineDetector()

    rospy.logwarn('>>> WL ready')
    rate = rospy.Rate(25)

    while not rospy.is_shutdown():
        det.update()

    rate.sleep()


if __name__ == '__main__':
    main()


