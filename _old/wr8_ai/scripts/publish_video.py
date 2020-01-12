#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Publish a video as ROS messages.
"""

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

# import camera_info_manager

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    """Publish a video as ROS messages.
    """
    # Patse arguments.
    parser = argparse.ArgumentParser(description="Convert video into a rosbag.")
    parser.add_argument("video_file", help="Input video.")
    parser.add_argument("-c", "--camera", default="camera", help="Camera name.")
    parser.add_argument("-f", "--frame_id", default="camera",
                        help="tf frame_id.")
    parser.add_argument("--width", type=np.int32, default="640",
                        help="Image width.")
    parser.add_argument("--height", type=np.int32, default="480",
                        help="Image height.")
    parser.add_argument("--info_url", default="file:///camera.yml",
                        help="Camera calibration url.")

    args = parser.parse_args()

    print "Publishing %s" % (args.video_file)

    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher("/" + args.camera + "/image_raw/compressed", CompressedImage,
                              queue_size=10)
    info_pub = rospy.Publisher("/" + args.camera + "/camera_info", CameraInfo,
                               queue_size=10)

    # info_manager = camera_info_manager.CameraInfoManager(cname=args.camera,
    #                                                      url=args.info_url,
    #                                                      namespace=args.camera)
    # info_manager.loadCameraInfo()

    # Open video.
    video = cv2.VideoCapture(args.video_file)
    # Get frame rate.
    # fps = video.get(cv2.cv.CV_CAP_PROP_FPS)
    rate = rospy.Rate(25)

    # Loop through video frames.
    while not rospy.is_shutdown():
        if not video.grab():
            # print 'Failed'
            continue

        tmp, img = video.retrieve()
        if not tmp:
            print "Could not grab frame."
            break

        img_out = np.empty((args.height, args.width, img.shape[2]))

        # Compute input/output aspect ratios.
        aspect_ratio_in = np.float(img.shape[1]) / np.float(img.shape[0])
        aspect_ratio_out = np.float(args.width) / np.float(args.height)

        if aspect_ratio_in > aspect_ratio_out:
            # Output is narrower than input -> crop left/right.
            rsz_factor = np.float(args.height) / np.float(img.shape[0])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = (img_rsz.shape[1] - args.width) / 2
            img_out = img_rsz[:, diff:-diff-1, :]
        elif aspect_ratio_in < aspect_ratio_out:
            # Output is wider than input -> crop top/bottom.
            rsz_factor = np.float(args.width) / np.float(img.shape[1])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = (img_rsz.shape[0] - args.height) / 2

            img_out = img_rsz[diff:-diff-1, :, :]
        else:
            # Resize image.
            img_out = cv2.resize(img, (args.height, args.width))

        assert img_out.shape[0:2] == (args.height, args.width)

        try:
            # Publish image.
            img_msg = CompressedImage #bridge.cv2_to_imgmsg(img_out, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = args.frame_id
            img_msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()
            img_pub.publish(img_msg)

            # Publish camera info.
            # info_msg = info_manager.getCameraInfo()
            # info_msg.header = img_msg.header
            # info_pub.publish(info_msg)
        except CvBridgeError as err:
            print err

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
