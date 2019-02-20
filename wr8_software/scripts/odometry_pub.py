#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import tf

odom_broadcaster = tf.TransformBroadcaster()
odom_pub = None

def callback(msg):
    current_time = rospy.Time.now()
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, msg.z)

    odom_broadcaster.sendTransform(
        (msg.x, msg.y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(msg.x, msg.y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    # publish the message
    odom_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    rospy.Subscriber('odom_pose', Point32, callback, queue_size = 10)
    rospy.spin()
