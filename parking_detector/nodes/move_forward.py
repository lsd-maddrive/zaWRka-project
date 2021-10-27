#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

def start_move():
    twist = Twist()
    while not rospy.is_shutdown():
        twist.linear.x = 0.3
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub.publish(twist)

        rate.sleep()

rospy.init_node('move_forward')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)

try:
    start_move()
except (rospy.ROSInterruptException, KeyboardInterrupt):
    rospy.logerr('Exception catched')