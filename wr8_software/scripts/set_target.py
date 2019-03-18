#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep
import tf
import math as m
import time

rospy.init_node('set_target')

def active_cb():
    print('active_cb()')

def feedback_cb(feedback):
    print('feedback_cb({})'.format(feedback))

def done_cb(state, result):
    print('done_cb({}, {})'.format(state, result))

# 0 = PENDING,    
# 1 = ACTIVE,     
# 2 = PREEMPTED,  
# 3 = SUCCEEDED,  
# 4 = ABORTED,    
# 5 = REJECTED,   
# 6 = LOST   

if __name__ == '__main__':

    print('Ready, go')

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 5
    goal.target_pose.pose.position.y = -1
    

    goal.target_pose.pose.position.x = 5
    goal.target_pose.pose.position.y = -2

    # goal.target_pose.pose.position.x = 3
    # goal.target_pose.pose.position.y = 0.5

    # goal.target_pose.pose.position.x = 7
    # goal.target_pose.pose.position.y = 0.3

    quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(-90))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    client.send_goal(goal,
                     active_cb=active_cb,
                     feedback_cb=feedback_cb,
                     done_cb=done_cb)

    print('Set goal')
    # print(goal)

    if 1:
        time.sleep(1)
        # PREEMPTED test
        client.cancel_all_goals()

    result = client.wait_for_result()

    print('Done: {}'.format(result))
    print('get_result() = {}'.format(client.get_result()))
    # Same as in done_cb
    print('get_state() = {}'.format(client.get_state()))
    
    # rospy.spin()
