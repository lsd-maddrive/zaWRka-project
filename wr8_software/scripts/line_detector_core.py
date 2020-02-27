#!/usr/bin/env python
import time

class LineDetector(object):
    def __init__(self):
        print "hello from LineDetector constructor"

    def process(self, image):
        print "hello from LineDetector process"
        return 1


if __name__ == "__main__":
    line_detector = LineDetector()

    WAITING_PERIOD = 1
    while not rospy.is_shutdown():
        result = line_detector.process("some_image")
        print "result is ", result
        time.sleep(WAITING_PERIOD)
