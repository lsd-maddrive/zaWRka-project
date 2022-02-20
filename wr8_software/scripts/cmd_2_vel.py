#!/usr/bin/env python2

"""
    Node converts cmd_vel to speed odometry based on model and parameters
"""

import rospy
from geometry_msgs.msg import Twist

PUB_TOPIC = 'cmd_odom_vel'
SUB_TOPIC = 'cmd_vel'

def main():
    rospy.init_node('cmd_2_vel')
    
    def cmd_cb(msg):
        src_twist = msg
        
        src_twist.linear.x *= 1.5
        
        pub.publish(src_twist)

    pub = rospy.Publisher(PUB_TOPIC, Twist, queue_size=1)
    rospy.Subscriber(SUB_TOPIC, Twist, cmd_cb, queue_size=1)

    rospy.spin()
    
    # while not rospy.is_shutdown():
        # rospy.

if __name__ == '__main__':
    exit(main())
