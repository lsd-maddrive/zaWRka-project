#!/usr/bin/env python
from __future__ import print_function

import rospy
rospy.init_node('action_cam_driver')

from sensor_msgs.msg import CompressedImage
img_pub_comp = rospy.Publisher('image_raw/compressed', CompressedImage, queue_size=1)
msgImg = CompressedImage()


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
img_pub = rospy.Publisher('image_raw', Image, queue_size=1)
bridge = CvBridge()


import numpy as np
import cv2
print(cv2.__version__)

import time


cap = cv2.VideoCapture('/dev/v4l/by-id/usb-KYE_Systems_Corp._USB_Camera_200901010001-video-index0')
if cap is None or not cap.isOpened(): 
    rospy.logerr('Failed to open camera')
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print(cap.get(cv2.CAP_PROP_FPS))
# cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

print(cap.get(cv2.CAP_PROP_CONVERT_RGB))

useFPS = True


def talker():

    start_time, end_time, full_time = 0, 0, 0
    frame_cntr   = 0
    measure_time = 1

    # rate = rospy.Rate(30) # 1hz
    while not rospy.is_shutdown():
        if useFPS:
            start_time = time.time()

        ret, frame = cap.read()
        if frame is None:
            print('Failed to read frame')
            break

        # frame = cv2.resize(frame, (640, 480))

        if useFPS:
            end_time = time.time()
            full_time   += end_time - start_time
            frame_cntr  += 1

            if full_time > measure_time:
                fps = float(frame_cntr) / full_time
                full_time, frame_cntr = 0, 0

                print('FPS: %g' % fps)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break

        msgImg.format = "jpeg"
        msgImg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        img_pub_comp.publish(msgImg)

        img_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
