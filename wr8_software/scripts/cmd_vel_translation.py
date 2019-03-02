#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf
import math as m
import numpy as np

cmd_pub = None

x_cmd = 0
z_cmd = 0

x_delta = 2
z_delta = 5

def callback(msg):
    global x_cmd, z_cmd
    sign = lambda x: m.copysign(1, x)

    if msg.linear.x == 0 and msg.angular.z == 0:
        x_cmd = 0
        z_cmd = 0;
    else:
        if msg.angular.z != 0:
            z_cmd += z_delta * sign(msg.angular.z)
        if msg.linear.x != 0:
            x_cmd += x_delta * sign(msg.linear.x)

    z_cmd = np.clip(z_cmd, -25, 25)
    x_cmd = np.clip(x_cmd, -100, 100)

    cmd = Twist()
    cmd.linear.x = x_cmd / 100.;
    cmd.angular.z = z_cmd;

    # print(cmd)

    cmd_pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_trans')

    cmd_pub = rospy.Publisher("output_cmd_vel", Twist, queue_size=50)
    rospy.Subscriber('input_cmd_vel', Twist, callback, queue_size=50)

    rospy.spin()
