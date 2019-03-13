#!/usr/bin/env python

import rospy
import json
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import wr8_ai.ncs as ncs

class ImageReceiverROS:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera", Image, self.callback, queue_size=1)

        self.cv_image = None

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

    def getImage(self):
        return self.cv_image


def main():

    rospy.init_node('test_signs')

    graph_path = rospy.get_param('~graph_path')
    config_path = rospy.get_param('~config_path')

    model = ncs.InferNCS(graph_path, fp16=False)
    rospy.loginfo('Model created')

    with open(config_path) as config_buffer:    
        config = json.load(config_buffer)
    rospy.loginfo('Config opened')

    img_rcvr = ImageReceiverROS()

    cntr = 0

    while not rospy.is_shutdown():
        img = img_rcvr.getImage()

        if img is None:
            rospy.sleep(1)
            cntr += 1
            if cntr > 3:
                rospy.logwarn('No image for 3 seconds...')
                cntr = 0
            continue



if __name__ == '__main__':
    main()



