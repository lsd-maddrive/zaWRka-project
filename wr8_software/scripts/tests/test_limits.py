#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf
import math as m

if __name__ == '__main__':

    rospy.init_node('test')

    
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(200))
    odom_broadcaster = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        odom_broadcaster.sendTransform(
            (0, 1, 0.),
            odom_quat,
            current_time,
            "test1",
            "test2"
        )

