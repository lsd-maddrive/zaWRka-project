#!/usr/bin/env python

import rospy
from wr8_ai.srv import ObjectDetection, ObjectDetectionRequest, ObjectDetectionResponse
import wr8_ai.utils as ut
from wr8_ai.yolo import fps
from cv_bridge import CvBridge, CvBridgeError
import time


def main():

    rospy.init_node('detection_srv_exec')

    img_rcvr = ut.ImageReceiverROS()
    bridge = CvBridge()

    poly_srv = rospy.ServiceProxy('detect_signs', ObjectDetection)

    rospy.loginfo('Ready for processing')

    rospy.wait_for_service('detect_signs')

    fps_meter = fps.FPSMeter()

    while not rospy.is_shutdown():
        image = img_rcvr.getImage()

        if image is None:
            rospy.sleep(1)
            continue

        try:
            start_time = time.time()

            ros_image = bridge.cv2_to_imgmsg(image, 'bgr8')
            req = ObjectDetectionRequest(image=ros_image)
            resp = poly_srv(req)

            fps_meter.update(time.time() - start_time)

            if fps_meter.milliseconds > 3000:
                fps_meter.print_statistics()
                fps_meter.reset()

            rospy.loginfo('Response: {}'.format(resp))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    main()
