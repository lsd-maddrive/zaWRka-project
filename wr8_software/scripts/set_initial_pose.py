#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import PoseWithCovarianceStamped


if __name__ == "__main__":
    # Init
    rospy.init_node('set_initial_pose')
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 1, latch=True)
    x = rospy.get_param('~start_x', 0)
    y = rospy.get_param('~start_y', 0)
    yaw = rospy.get_param('~start_yaw', 0)
    time.sleep(1)

    # Create msg and publish
    msg = PoseWithCovarianceStamped()
    now = rospy.get_rostime()
    msg.header.stamp.secs = now.secs
    msg.header.stamp.nsecs = now.nsecs
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    pub.publish(msg)

    # Wait few seconds
    time.sleep(90)
    exit(0)