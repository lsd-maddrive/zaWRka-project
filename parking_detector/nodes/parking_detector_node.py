#!/usr/bin/env python
from __future__ import print_function

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from detector.detector import Detector
from tf.transformations import quaternion_from_euler

parking_pub = rospy.Publisher('parking_goal', PoseStamped, queue_size=1, latch=True)
TIME = 0
TRIGGER = False
POINTS = []

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('/home/artemon12/PycharmProjects/parking_diploma/pics/output12.avi', fourcc, 20.0, (1280,1024))

def callback(data):
    bridge = CvBridge()
    height, width = 1024, 1280

    detector = Detector(height=height, width=width, is_live=False)
    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    data = detector.detection_by_frame(image)
    print('parking center found =  ', data['found'])
    #cv2.imwrite('/home/artemon12/PycharmProjects/parking_diploma/pics/lot_res1.jpg', image)
    out.write(image)
    
    if data['found']:
        quat = quaternion_from_euler(0, 0, np.pi/2)

        x_posititon = 0.7188 + data['parking_params']['x']
        y_posititon = -1*(0.7967 + data['parking_params']['y'])

        position = PoseStamped()
        position.header.seq = 0
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = 'map'
        position.pose.position = Point(x_posititon, y_posititon, 0)
        position.pose.orientation = Quaternion(*quat)

    if data['found'] and TRIGGER is not True:
        global TIME
        if TIME is 0:
            TIME = data['time']

        POINTS.append(data['parking_params']['center'])

        if data['time'] - TIME > 1.5:
            global TRIGGER
            global POINTS
            average = np.std(POINTS)
            if average < 5:
                parking_pub.publish(position)
                TRIGGER = True


if __name__ == '__main__':
    rospy.init_node('parking_detector')

    rospy.Subscriber('/signs_camera/image_raw', Image, callback)

    rospy.spin()

    out.release()
