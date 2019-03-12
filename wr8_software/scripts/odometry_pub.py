#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf
import math as m

odom_broadcaster = tf.TransformBroadcaster()
odom_pub = None

def callback(msg):
    x = msg.data[0]
    y = msg.data[1]
    teta_deg = msg.data[2]
    vx = msg.data[3]
    uz = msg.data[4]

    current_time = rospy.Time.now()
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(teta_deg))

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, uz))

    # publish the message
    odom_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    rospy.Subscriber('odom_pose', Float32MultiArray, callback, queue_size = 10)
    rospy.spin()
