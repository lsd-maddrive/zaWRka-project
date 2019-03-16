#!/usr/bin/env python

import rospy
from wr8_ai.srv import ObjectDetection, ObjectDetectionResponse
from cv_bridge import CvBridge, CvBridgeError

import wr8_ai.detector_ncs as det

rospy.init_node('yolo_detection')

graph_path = rospy.get_param('~graph_path')
config_path = rospy.get_param('~config_path')

detector = det.DetectorNCS()
if not detector.init(0, graph_path, config_path):
    rospy.logerr('Failed to initialize detector')        

def handle_srv(self, req):
    try:
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
    except CvBridgeError as e:
        rospy.logwarn(e)

    resp = ObjectDetectionResponse()

    boxes, box_img = detector.get_signs(cv_img=image)

    resp.bboxes = boxes

    return resp


def main():

    rospy.Service('detect_signs', ObjectDetection, handle_srv)
    rospy.loginfo("Ready to detect signs!")

    rospy.spin()


if __name__ == '__main__':
    main()

