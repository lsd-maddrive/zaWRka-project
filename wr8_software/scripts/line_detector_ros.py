#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image

from line_detector_core import LineDetector

NODE_NAME = "white_line_detector"
IMAGE_SUB_TOPIC = "signs_camera/image_raw"
STATUS_PUB_TOPIC = "wl_status"

class LineDetectorRos(object):
    def __init__(self):
        self.detector = LineDetector()

        self.msg = None
        self.image_sub = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, self._image_cb, queue_size=10)
        self.status_pub = rospy.Publisher(STATUS_PUB_TOPIC, UInt8, queue_size=5)

    def process(self):
        result = self.detector.process(self.msg)

        self.status_pub.publish(result)
        rospy.logdebug("image has been processed and result has been received: %d", result)

    def _image_cb(self, msg):
        self.msg = msg
        rospy.logdebug("Heard image.")
        

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    line_detector = LineDetectorRos()
    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        line_detector.process()
        RATE.sleep()
