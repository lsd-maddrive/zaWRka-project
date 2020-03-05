#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from line_detector_core import LineDetector

NODE_NAME = "white_line_detector"
IMAGE_SUB_TOPIC = "stereo_camera_converted/left/image_raw"
STATUS_PUB_TOPIC = "wl_status"
IMG_PUB_TOPIC = "img_from_core"

class LineDetectorRos(object):
    def __init__(self, angle_threshold, similarity_threshold):
        self.detector = LineDetector(angle_threshold, similarity_threshold)
        self.bridge = CvBridge()

        self.msg = None
        self.image_sub = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, self._image_cb, queue_size=10)
        self.status_pub = rospy.Publisher(STATUS_PUB_TOPIC, UInt8, queue_size=5)
        self.img_pub_to_rviz = rospy.Publisher(IMG_PUB_TOPIC, Image, queue_size=5)

    def process(self):
        if self.msg is None:
            print "None"
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.msg, "bgra8")
            result, cv_img_from_core = self.detector.process(cv_image)
            ros_img_from_core = self.bridge.cv2_to_imgmsg(cv_img_from_core, encoding="passthrough")
            self.img_pub_to_rviz.publish(ros_img_from_core)
            self.status_pub.publish(result)
            rospy.logdebug("image has been processed and result has been received: %d", result)
        except CvBridgeError as e:
            print(e)

    def _image_cb(self, msg):
        self.msg = msg
        rospy.logdebug("Heard image.")
        

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    line_detector = LineDetectorRos(math.pi/32, 10)
    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        line_detector.process()
        RATE.sleep()
