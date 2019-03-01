#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf
import math as m

if __name__ == '__main__':

    current_time = rospy.Time.now()
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(270))

    
    while not rospy.is_shutdown():
    	odom_broadcaster.sendTransform(
	        (msg.x, msg.y, 0.),
	        odom_quat,
	        current_time,
	        "test1",
	        "test2"
	    )

