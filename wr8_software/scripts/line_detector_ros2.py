#!/usr/bin/env python2
import rospy
import math
from threading import Timer
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image

from line_detector_core import LineDetector

class LineDetectorRos(object):
    def __init__(self, receiver, publisher):
        img_top_cropping_border = rospy.get_param('~white_line_detector/img_top_cropping_border')
        img_bot_cropping_border = rospy.get_param('~white_line_detector/img_bot_cropping_border')
        min_theta = rospy.get_param('~white_line_detector/min_thetta')
        max_theta = rospy.get_param('~white_line_detector/max_thetta')
        rho_min_offset = rospy.get_param('~white_line_detector/rho_min_offset')
        rho_max_offset = rospy.get_param('~white_line_detector/rho_max_offset')
        theta_similarity_threshold = rospy.get_param('~white_line_detector/theta_similarity_threshold')
        min_vote = rospy.get_param('~white_line_detector/min_vote')

        frequency = rospy.get_param('~white_line_detector/frequency')
        self._img_publisher = publisher
        self._img_receiver = receiver
        self._detector = LineDetector(img_top_cropping_border,
                                      img_bot_cropping_border,
                                      min_theta,
                                      max_theta,
                                      rho_min_offset,
                                      rho_max_offset,
                                      theta_similarity_threshold,
                                      min_vote,
                                      draw_lines=self._img_publisher.isEnabled())
        self._enable = False
        self._PERIOD = 1.0 / frequency if frequency != 0 else 1.0
        self._timer = None
        self._added_lines = None

    def start(self):
        self._enable = True
        if self._timer is None:
            self._timer = Timer(self._PERIOD, self._process)
            self._timer.start()

    def stop(self):
        self._enable = False
        if self._timer is not None:
            self._timer.cancel()

    def get_lines(self):
        return self._added_lines

    def _process(self):
        if self._enable is True:
            self._timer = Timer(self._PERIOD, self._process)
            self._timer.start()
        
        img = self._img_receiver.get()
        if img is None:
            rospy.logwarn("There is no input image. Is topic correct?")
            return None
        
        self._added_lines, cv_img_from_core = self._detector.process(img)
        
        # if self._img_publisher.isEnabled():
        #     if cv_img_from_core is not None:
        #         self._img_publisher.send(cv_img_from_core)
            
        return self._added_lines

    def _image_cb(self, msg):
        self._msg = msg
        

if __name__ == "__main__":
    NODE_NAME = "white_line_detector"
    STATUS_PUB_TOPIC = "wl_status"

    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    rospy.set_param('~white_line_detector/camera_topic', 'cam_bottom')
    rospy.set_param('~white_line_detector/img_topic', None)

    line_detector = LineDetectorRos()
    line_detector.start()

    status_pub = rospy.Publisher(STATUS_PUB_TOPIC, UInt8, queue_size=5)

    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        added_lines = line_detector.get_lines()
        if added_lines is not None:
            rospy.logdebug("Image has been processed, number of lines = %d", len(added_lines))
            status_pub.publish(len(added_lines))
        RATE.sleep()
