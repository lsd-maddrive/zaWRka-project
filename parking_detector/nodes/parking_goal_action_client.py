#! /usr/bin/env python
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID


def callback(data):
    print('Parking lot center recieved')
    print(data)
    result = parking_goal_action_client(data)
    print(result)

def parking_goal_action_client(data):
    # Creates the SimpleActionClient, passing the type of the action
    print('Starting client')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal()
    goal.target_pose = data

    # Sends the goal to the action server.
    client.send_goal(goal)

    print('GOAL SENDED TO SERVER')  
    print(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('parking_goal_action_client')
        rospy.Subscriber('parking_goal', PoseStamped, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)