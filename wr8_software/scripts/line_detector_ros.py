#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from line_detector_core import LineDetector

class LineDetectorRos(object):
    def __init__(self, min_theta, max_theta, similarity_threshold, img_sub_topic, img_pub_topic = None):
        self.detector = LineDetector(min_theta, max_theta, similarity_threshold, img_pub_topic is not None)
        self.bridge = CvBridge()
        self.msg = None

        rospy.Subscriber(img_sub_topic, Image, self._image_cb, queue_size=10)
        if img_pub_topic != None:
            self.img_pub = rospy.Publisher(img_pub_topic, Image, queue_size=5)
        else:
            self.img_pub = None

    def process(self):
        if self.msg is None:
            rospy.logwarn("There is no input image. Is topic correct?")
            return None
        try:
            cv_image_from_ros = self.bridge.imgmsg_to_cv2(self.msg, "bgra8")
            added_lines, cv_img_from_core = self.detector.process(cv_image_from_ros)
            if self.img_pub is not None and cv_img_from_core is not None:
                img_to_ros = self.bridge.cv2_to_imgmsg(cv_img_from_core, encoding="passthrough")
                self.img_pub.publish(img_to_ros)
        except CvBridgeError as e:
            rospy.logerr(e)
        return added_lines

    def _image_cb(self, msg):
        self.msg = msg
        rospy.logdebug("Heard image.")
        

if __name__ == "__main__":
    NODE_NAME = "white_line_detector"
    IMG_SUB_TOPIC = "stereo_camera/left/image_raw" # or "converted"
    IMG_PUB_TOPIC = "img_from_core"
    STATUS_PUB_TOPIC = "wl_status"

    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    line_detector = LineDetectorRos(math.pi*7/16, math.pi*9/16, 10, IMG_SUB_TOPIC, IMG_PUB_TOPIC)

    status_pub = rospy.Publisher(STATUS_PUB_TOPIC, UInt8, queue_size=5)

    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        added_lines = line_detector.process()
        if added_lines != None:
            rospy.logdebug("Image has been processed, number of lines = %d", len(added_lines))
            status_pub.publish(len(added_lines))
        RATE.sleep()
