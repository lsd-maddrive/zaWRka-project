#!/usr/bin/env python

import rospy
import json
import cv2
import wr8_ai.ncs as ncs
import wr8_ai.utils as ut
from wr8_ai.yolo import yolo
from wr8_ai.srv import ObjectDetection, ObjectDetectionResponse
from wr8_ai.msg import BoundingBox
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


class YOLO_Detector:
    def __init__(self, graph_path, config_path):
        self.bridge = CvBridge()

        self.model = ncs.InferNCS(graph_path, fp16=False)

        if not self.model.is_opened():
            rospy.logerr('Failed to init device')
            return

        with open(config_path) as config_buffer:
            config = json.load(config_buffer)
        rospy.loginfo('Config opened')

        self.labels = ['brick', 'forward', 'forward and left', 'forward and right', 'left', 'right']
        anchors = config['model']['anchors']

        net_h, net_w = config['model']['infer_shape']
        obj_thresh, nms_thresh = 0.5, 0.45

        self.inferer = yolo.YOLO(self.model, net_h, net_w, anchors, obj_thresh, nms_thresh)

        rospy.Service('detect_signs', ObjectDetection, self.handle_srv)
        rospy.loginfo("Ready to detect signs!")

    def handle_srv(self, req):
        try:
            cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)

        boxes = self.inferer.make_infer([cv_image])[0]

        resp = ObjectDetectionResponse()

        resp.bboxes = []
        for box in boxes:
            resp.bboxes += [ut.yolo_bbox_2_ros_bbox(box, self.labels)]

        return resp

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node('yolo_detection')

    graph_path = rospy.get_param('~graph_path')
    config_path = rospy.get_param('~config_path')

    yolo_srv = YOLO_Detector(graph_path, config_path)
    yolo_srv.spin()


if __name__ == '__main__':
    main()

