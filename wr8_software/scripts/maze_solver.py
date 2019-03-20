#!/usr/bin/env python

import numpy as np
import time
import random

from graph_path.maze import *
from graph_path.car_state import *

# import graph_path.gui

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf


class MazeSolver:
    def __init__(self, render_mode=False):
        structure = [[0, 0, 0, 0, 0, 0, 0],
                     [0, 8, 0, 8, 0, 8, 0],
                     [0, 8, 0, 8, 0, 0, 0],
                     [0, 0, 0, 0, 0, 8, 8],
                     [0, 8, 0, 8, 0, 8, 8],
                     [0, 8, 0, 8, 0, 0, 2],
                     [1, 8, 0, 0, 0, 8, 8]]
        structure = np.array(structure, np.uint8)
        self.maze = Maze(structure)

        self.render_mode = render_mode

        if self.render_mode:
            self.maze.render_maze()
        
        self.maze.nextPreprocessing()

        self.car = CarState(self.maze)

        if self.render_mode:
            self.car.renderCarPosition()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.is_goal_completed = True
        self.is_goal_failed = False
        self.is_maze_completed = False
        self.end_pending = False

        rospy.loginfo('MazeSolver initialized')

    def __del__(self):
        pass

    def init(self):
        self.client.wait_for_server()

    def done_cb(self, state, result):
        if state == GoalStatus.SUCCEEDED:
            self.car.move2Target(self.tNode)
            self.is_goal_completed = True

            if self.end_pending:
                self.is_maze_completed = True
        else:
            self.is_goal_failed = True

    def get_goal_result(self):
        return self.goal_state

    def set_next_cross_goal(self):
        if self.is_maze_completed:
            return

        if self.render_mode:
            self.maze.render_maze()

        if not self.car.solveMazeAStar():
            print('Failed to find path...')
            return False

        self.tNode = self.car.getNextTarget()
        # print('Next node: {}'.format(tNode))

        rospy.loginfo('Current node: {}'.format(self.car.cNode))
        rospy.loginfo('Set target: {}'.format(self.tNode))

        shift_dir = { 
                from_direction_letters[0] : Point(0.35, -1),
                from_direction_letters[1] : Point(-1, -0.35),
                from_direction_letters[2] : Point(-0.35, 1),
                from_direction_letters[3] : Point(1, 0.35)
            }        

        if self.car.isTargetEnding(self.tNode):
            tPoint = self.tNode.coord + Point(0.8, 0)
            self.end_pending = True
        else:
            tPoint = self.tNode.coord - shift_dir[self.tNode.dirLetr]
            self.end_pending = False

        self.tPointDir = PointDir(tPoint.x, tPoint.y, self.tNode.dirLetr)

        self.set_maze_goal(self.tPointDir)

        rospy.loginfo('Goal is sent')

        return True

    def cancel_current_goal(self):
        self.cancel_goal(self.client)

    def recover_current_goal(self):
        self.set_maze_goal(self.tPointDir)

    def is_sign_required(self):
        return self.car.isSignRequired()

    def set_vision_sign(self, car_sign):
        self.car.updateSignInfo(car_sign)

    # 0 = PENDING,    
    # 1 = ACTIVE,     
    # 2 = PREEMPTED,  
    # 3 = SUCCEEDED,  
    # 4 = ABORTED,    
    # 5 = REJECTED,   
    # 6 = LOST   
    def get_goal_state(self):
        return self.client.get_state()

    def set_maze_goal(self, tPointDir):
        tPoint_ros_x, tPoint_ros_y, tPoint_angle = tPointDir.get_ros_point()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = tPoint_ros_x
        goal.target_pose.pose.position.y = tPoint_ros_y

        quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(tPoint_angle))
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        
        self.client.send_goal(goal, done_cb=self.done_cb)

        self.is_goal_completed = False
        self.is_goal_failed = False

    def set_direct_goal(self, ros_pnt, ros_angle_deg):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = ros_pnt[0]
        goal.target_pose.pose.position.y = ros_pnt[1]

        quat = tf.transformations.quaternion_from_euler(0, 0, m.radians(ros_angle_deg))
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        
        self.client.send_goal(goal, done_cb=self.done_cb)
        
        self.is_goal_completed = False
        self.is_goal_failed = False

    def cancel_goal(self):
        self.client.cancel_all_goals()

    def wait_4_goal_complete(self):
        self.client.wait_for_result()


def main():
    import pygame
    from pygame.locals import *

    pygame.init()

    ### ROS
    rospy.init_node("maze_solver")

    solver = MazeSolver(render_mode=True)
    solver.init()

    solver.set_next_cross_goal()



    while not rospy.is_shutdown():

        if not solver.is_maze_completed:

            if solver.is_goal_completed:
                # Signs test
                if solver.is_sign_required():
                    seleceted_sign = int(random.random() * 6)

                    # seleceted_sign = SIGNS_NONE

                    solver.set_vision_sign(seleceted_sign)

                    rospy.loginfo('Set selected sign: {} [{}]'.format(sign_names[seleceted_sign], seleceted_sign))

                if not solver.set_next_cross_goal():
                    rospy.loginfo('Failed to get maze path')

                    solver.set_vision_sign(SIGNS_NONE)
                    if not solver.set_next_cross_goal():
                        # Check again, failed
                        rospy.loginfo('Failed to get maze path (2)')
                        exit(1)

                continue
            else:
                rospy.loginfo('Waiting for complete')
                time.sleep(1)

        else:
            
            solver.set_direct_goal((13.0, -14.6), 0)
            solver.wait_4_goal_complete()

            solver.set_direct_goal((14.9, 0.8), 90)
            solver.wait_4_goal_complete()

            break

        # Pygame checks
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == 32: # Space
                    exit()

    pygame.quit()

if __name__ == '__main__':
    main()
