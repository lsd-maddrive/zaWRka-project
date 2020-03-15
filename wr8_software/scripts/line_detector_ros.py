#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from line_detector_core import LineDetector

class LineDetectorRos(object):
    def __init__(self, min_theta, max_theta, similarity_threshold, img_sub_topic, img_pub_topic = None):
        self._detector = LineDetector(min_theta, max_theta, similarity_threshold, img_pub_topic is not None)
        self._bridge = CvBridge()
        self._msg = None
        self._enable = False
        self._added_lines = None

        rospy.Subscriber(img_sub_topic, Image, self._image_cb, queue_size=10)
        if img_pub_topic != None:
            self._img_pub = rospy.Publisher(img_pub_topic, Image, queue_size=5)
        else:
            self._img_pub = None

    def enable(self):
        self._enable = True

    def disable(self):
        self._enable = False

    def get_lines(self):
        return self._added_lines

    def _process(self):
        if self._msg is None:
            rospy.logwarn("There is no input image. Is topic correct?")
            return None
        try:
            cv_image_from_ros = self._bridge.imgmsg_to_cv2(self._msg, "bgr8")
            self._added_lines, cv_img_from_core = self._detector.process(cv_image_from_ros)
            if self._img_pub is not None and cv_img_from_core is not None:
                img_to_ros = self._bridge.cv2_to_imgmsg(cv_img_from_core)
                self._img_pub.publish(img_to_ros)
        except CvBridgeError as e:
            rospy.logerr(e)
        return self._added_lines

    def _image_cb(self, msg):
        self._msg = msg
        if self._enable == True:
            self._process()
            rospy.logdebug("Heard image and process it.")
        else:
            rospy.logdebug("Heard image but line detector is disabled.")
        

if __name__ == "__main__":
    NODE_NAME = "white_line_detector"
    IMG_SUB_TOPIC = "stereo_camera_converted/left/image_raw"
    IMG_PUB_TOPIC = "img_from_core"
    STATUS_PUB_TOPIC = "wl_status"

    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    line_detector = LineDetectorRos(math.pi*7/16, math.pi*9/16, 10, IMG_SUB_TOPIC, IMG_PUB_TOPIC)
    line_detector.enable()

    status_pub = rospy.Publisher(STATUS_PUB_TOPIC, UInt8, queue_size=5)

    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        added_lines = line_detector.get_lines()
        if added_lines is not None:
            rospy.logdebug("Image has been processed, number of lines = %d", len(added_lines))
            status_pub.publish(len(added_lines))
        RATE.sleep()
