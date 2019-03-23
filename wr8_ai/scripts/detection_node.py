#!/usr/bin/env python

import rospy
from wr8_ai.srv import ObjectDetection, ObjectDetectionResponse
from cv_bridge import CvBridge, CvBridgeError
from wr8_ai.detectors import SignsDetector

def main():
    rospy.init_node('yolo_detection')

    signs_detector = SignsDetector()
    if not signs_detector.init():
        rospy.logerr('Failed to initilize detector')
        signs_detector = None
        exit(1)

    rospy.logwarn('>>> SNN ready')

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
    	signs_detector.update()

    rate.sleep()

if __name__ == '__main__':
    main()

