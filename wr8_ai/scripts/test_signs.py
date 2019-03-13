#!/usr/bin/env python

import rospy
import json
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

import wr8_ai.ncs as ncs
from wr8_ai.yolo import yolo
from wr8_ai.yolo import bbox
from wr8_ai.yolo import fps

import time


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

    fps_meter = fps.FPSMeter()

    model = ncs.InferNCS(graph_path, fp16=False)

    if not model.is_opened():
        rospy.logerr('Failed to init device')
        return

    rospy.loginfo('Model created')

    with open(config_path) as config_buffer:    
        config = json.load(config_buffer)
    rospy.loginfo('Config opened')

    labels = ['brick', 'forward', 'forward and left', 'forward and right', 'left', 'right']
    anchors = config['model']['anchors']

    net_h, net_w = config['model']['infer_shape']
    obj_thresh, nms_thresh = 0.5, 0.45

    inferer = yolo.YOLO(model, net_h, net_w, anchors, obj_thresh, nms_thresh)

    img_rcvr = ImageReceiverROS()

    skip_cntr = 0

    while not rospy.is_shutdown():
        image = img_rcvr.getImage()

        if image is None:
            rospy.sleep(1)
            skip_cntr += 1
            if skip_cntr > 3:
                rospy.logwarn('No image for 3 seconds...')
                skip_cntr = 0
            continue

        start = time.time()

        boxes = inferer.make_infer([image])[0]

        fps_meter.update(time.time() - start)

        if fps_meter.milliseconds > 3000:
            fps_meter.print_statistics()
            fps_meter.reset()

        bbox.draw_boxes(image, boxes, labels, obj_thresh)

        cv2.imshow('2', image)
        key = cv2.waitKey(10)
        if key == 27:
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
