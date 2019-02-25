#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf



if __name__ == '__main__':
    rospy.init_node('sample_odometry')

    smpl_x = 0
    # rate = rospy.Rate(1)

    # for i in range(10):
    current_time = 

    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        (0, 0, 0.),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "odom",
        "map"
    )
