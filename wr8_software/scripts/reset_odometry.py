#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf

if __name__ == '__main__':
    rospy.init_node('rst_odom')

    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        (0, 0, 0.),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "odom",
        "map"
    )
