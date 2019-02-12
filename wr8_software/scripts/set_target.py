#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import tf
import math as m


rospy.init_node('set_target')

if __name__ == '__main__':

    print('Ready, go')

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if 1:
	    goal.target_pose.pose.position.x = 7.5
	    goal.target_pose.pose.position.y = -6

	    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	    goal.target_pose.pose.orientation.x = quat[0]
	    goal.target_pose.pose.orientation.y = quat[1]
	    goal.target_pose.pose.orientation.z = quat[2]
	    goal.target_pose.pose.orientation.w = quat[3]
    else:
	    goal.target_pose.pose.position.x = 3
	    goal.target_pose.pose.position.y = -1

	    quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(-90))
	    goal.target_pose.pose.orientation.x = quat[0]
	    goal.target_pose.pose.orientation.y = quat[1]
	    goal.target_pose.pose.orientation.z = quat[2]
	    goal.target_pose.pose.orientation.w = quat[3]

    client.send_goal(goal)

    print('Set goal')
    print(goal)

    # client.wait_for_result()

    print('Done!')
    # rospy.spin()
