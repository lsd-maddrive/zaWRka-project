#!/usr/bin/env python

import numpy as np
import pygame
from pygame.locals import *
import time

from graph_path.maze import *
from graph_path.car_state import *

import graph_path.gui

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

# local TF -> ROS: (x1, y1) = (2*y, -2*x)
# ROS -> local TF: (x, y) = (-y1/2, x1/2)

def move_sim(tNode, mvClient):
    # print(tNode.dirLetr)
    # print(tNode.coord.get_array())

    tPoint = tNode.coord - from_directions[tNode.dirLetr]
    tPoint_angle = direction_angle[tNode.dirLetr]

    tPoint_ros_x =  2 * tPoint.y
    tPoint_ros_y = -2 * tPoint.x


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
    
    mvClient.send_goal(goal)

    print('Set goal')
    print(goal)

    mvClient.wait_for_result()

def main(args=None):

    pygame.init()

    structure = [[0, 0, 0, 0, 0, 0, 0],
                 [0, 8, 0, 8, 0, 8, 0],
                 [0, 8, 0, 8, 0, 0, 0],
                 [0, 0, 0, 0, 0, 8, 8],
                 [0, 8, 0, 8, 0, 8, 8],
                 [0, 8, 0, 8, 0, 0, 2],
                 [1, 8, 0, 0, 0, 8, 8]]
    structure = np.array(structure, np.uint8)
    maze = Maze(structure)

    maze.render_maze()
    maze.nextPreprocessing()

    car = CarState(maze)
    car.renderCarPosition()

    ### ROS
    rospy.init_node("maze_solver")

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    while not rospy.is_shutdown():

        maze.render_maze()

        if not car.solveMazeAStar():
            gui.notifyUser('Failed to find path =(')
            print('Failed to find path...')
            break

        tNode = car.getNextTarget()
        # print('Next node: {}'.format(tNode))

        move_sim(tNode, client)
        car.move2Target(tNode)

        if car.isEndReached():
            gui.notifyUser('Woohoo, we did it!')
            print('Woohoo, we did it!')
            break

        # Update display
        car.renderPath()
        car.renderCarPosition()
        print('Moved to {}'.format(tNode))
        
        if car.updateSignsInfo() < 0:
            break

        time.sleep(0.5)

        # Pygame checks
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == 32: # Space
                    exit()

    pygame.quit()


if __name__ == '__main__':
    main()
