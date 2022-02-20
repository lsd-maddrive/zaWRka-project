# Explicit import
ROS_PACKAGES_PATH = '/opt/ros/melodic/lib/python2.7/dist-packages/'

import os
CV_BRIDGE_PATH = os.path.join(ROS_PACKAGES_PATH, 'cv_bridge')
if not os.path.exists(CV_BRIDGE_PATH):
    raise Exception('Failed to find cv_bridge from ROS by path {}, check ros-melodic-cv-bridge module'.format(CV_BRIDGE_PATH))
else:
    print('cv_bridge in ROS exists by path {}'.format(CV_BRIDGE_PATH))

import sys
sys.path.insert(0, ROS_PACKAGES_PATH)

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import rospy
import cv2
import numpy as np


class ImageTopicReceiver(object):
    def __init__(self, topic, hFlip=False, vFlip=False, latched=False, **kwargs):
        self.topic = topic
        self.hFlip = hFlip
        self.vFlip = vFlip
        self.latched = latched
        
        self._bridge = CvBridge()
        
        rospy.Subscriber(self.topic, Image, self._cb, **kwargs)
        self.msg = None

    def _cb(self, msg):
        self.msg = msg
        
    def _convert_ros_img_to_detector_msg(self, img):
        img = self._bridge.imgmsg_to_cv2(img, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # img = img[::-1, ::-1]
        return img
        
    def get(self):
        if self.msg is None:
            return None
        
        try:
            img = self._bridge.imgmsg_to_cv2(self.msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return None
        
        if not self.latched:
            self.msg = None
        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Just to optimize to operations
        if self.hFlip and self.vFlip:
            img = img[::-1, ::-1]
        elif self.hFlip:
            img = img[:, ::-1]
        elif self.vFlip:
            img = img[::-1]
        
        return img
        

class CompressedImageTopicPublisher(object):
    def __init__(self, topic, **kwargs):
        self.topic = topic
        if topic is not None:        
            self._pub = rospy.Publisher(topic, CompressedImage, **kwargs)
        else:
            self._pub = None
        
    def isEnabled(self):
        return self._pub is not None
        
    def send(self, img_rgb):
        if self._pub is None:
            return
        
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img_rgb)[1]).tostring()
        
        self._pub.publish(msg)
        

class ImageTopicPublisher(object):
    def __init__(self, topic, **kwargs):
        self.topic = topic
        if topic is not None:        
            self._pub = rospy.Publisher(topic, Image, **kwargs)
        else:
            self._pub = None
        
        self._bridge = CvBridge()
        
    def isEnabled(self):
        return self._pub is not None
        
    def send(self, img_rgb):
        if self._pub is None:
            return
        
        img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        msg = self._bridge.cv2_to_imgmsg(img)
        self._pub.publish(msg)
        