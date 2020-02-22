#!/usr/bin/env python
import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import math
import numpy as np
import pygame
import time
from enum import Enum

from maze import MazePoint, PointDir, Maze

# Constants
class State(Enum):
    IDLE = 1

    MAZE_PROCESS = 2
    MAZE_WAIT_TL = 3
    MAZE_WAIT_SIGNS = 3
    MAZE_TARGET_REACHED = 4

    SPEED_PROCESS = 5
    SPEED_REACHED = 6

    PARKING_PROCESS = 7
    PARKING_REACHED = 8

NODE_NAME = "solver_node"
PATH_PUB_NAME = "path"
GOAL_PUB_NAME = "move_base/goal"
HARDWARE_SUB_TOPIC = "hardware_status"

POSE_SUB_TOPIC = "pose"
FRAME_ID = "map"

# Params
MAZE_TARGET_X = 1
MAZE_TARGET_Y = 2
OFFSET_X = 19
OFFSET_Y = 19
OFFSET_Z = 3.14
CELL_SZ = 2

# Global variables
state = State.IDLE
hardware_status = False
white_line_status = False

# Coordinate transform from MazePoint to MazePoint
def maze_to_gz(p):
    map_x = p.x * CELL_SZ + 0.5*CELL_SZ
    map_y = p.y * CELL_SZ + 0.5*CELL_SZ
    return MazePoint(map_x, map_y)

def gz_to_maze(p):
    maze_x = round((p.x - 0.5*CELL_SZ) / CELL_SZ)
    maze_y = round((p.y - 0.5*CELL_SZ) / CELL_SZ)
    return MazePoint(maze_x, maze_y)

def gz_to_map(p):
    gz_x = (p.x - OFFSET_X) * math.cos(-OFFSET_Z) - (p.y - OFFSET_Y) * math.sin(-OFFSET_Z)
    gz_y = (p.x - OFFSET_X) * math.sin(-OFFSET_Z) + (p.y - OFFSET_Y) * math.cos(-OFFSET_Z)
    return MazePoint(gz_x, gz_y)

def map_to_gz(p):
    map_x = (p.x - OFFSET_X) * math.cos(-OFFSET_Z) - (p.y - OFFSET_Y) * math.sin(-OFFSET_Z)
    map_y = (p.x - OFFSET_X) * math.sin(-OFFSET_Z) + (p.y - OFFSET_Y) * math.cos(-OFFSET_Z)
    return MazePoint(map_x, map_y)

def maze_to_map(p):
    return gz_to_map(maze_to_gz(p))

def map_to_maze(p):
    return gz_to_maze(map_to_gz(p))


# ROS Callbacks
def hardware_callback(msg):
    rospy.logdebug("I heard hardware %s", str(msg.data))
    global hardware_status
    if msg.data == 0:
        hardware_status = False
    elif msg.data == 1:
        hardware_status = True


def init_maze(crnt):
    structure = [[8, 8, 8, 0, 0, 0, 0, 1, 0, 0],
                 [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                 [8, 8, 8, 0, 0, 0, 0, 0, 8, 0],
                 [8, 8, 8, 0, 8, 0, 8, 0, 0, 0],
                 [8, 8, 8, 0, 8, 0, 8, 8, 8, 0],
                 [8, 8, 0, 0, 0, 0, 0, 0, 0, 0],
                 [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                 [8, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                 [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                 [8, 8, 0, 0, 0, 0, 0, 0, 0, 0]]

    structure = np.array(structure, np.uint8)
    maze = Maze(structure)
    maze.set_target(MazePoint(MAZE_TARGET_X, MAZE_TARGET_Y))
    initial_pose = crnt #Maze.ext_point_2_pointdir([OFFSET_Y, -OFFSET_X], 0)
    print Maze.ext_point_2_pointdir([OFFSET_Y, -OFFSET_X], 0)
    print PointDir(initial_pose.x, initial_pose.y, 0)
    maze.set_state(PointDir(initial_pose.x, initial_pose.y, 'D'))

    return maze

def render_maze(maze):
    pygame.init()
    maze.render_maze()

def pub_path(nodes):
    path = Path()
    path.header.seq = 1
    path.header.frame_id = FRAME_ID
    for i in range(0, len(nodes)):
        pose = PoseStamped()
        pose.header.seq = 1
        pose.header.frame_id = FRAME_ID

        point = maze_to_map(nodes[i].coord)
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        rospy.logdebug("path pose is %s", str(point))
        path.poses.append(pose)
    PATH_PUB.publish(path)


def get_maze_current_pose(listener):
    trans = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
    #except:
    #    point = None
    return point

def pose_callback(msg):
    rospy.logdebug("I heard smth")


def determinate_state(crnt):
    """
    crnt - maze current pose
    """
    state = State
    if hardware_status is False or crnt is None:
        state = State.IDLE
    elif crnt.x >= 3 or (crnt.x >= 2 and crnt.y <= 5):
        state = State.MAZE_PROCESS
    elif crnt.y <= 5:
        state = State.SPEED_PROCESS
    elif crnt.y >= 6:
        state = State.PARKING_PROCESS
    
    return state

def process_maze(maze, crnt):
    path = maze.get_path()
    local = maze.get_local_target()
    goal_point = None
    if crnt is None:
        rospy.logerr("Maze processing error: crnt pose must exists!")
    elif local is None:
        rospy.logerr("Maze processing error: local target must exists!")
    else:
        local = local.coord
        pub_path(path)
        if local.x == crnt.x and local.y == crnt.y:
            rospy.logdebug("New path:")
            for node in path:
                rospy.logdebug("- %s", node)
            
            maze.next_local_target()
        goal_point = maze_to_map(path[-1].coord)
        send_goal(GOAL_CLIENT, goal_point)
    return goal_point

def process_speed():
    return maze_to_map(MazePoint(1, 7))

def process_parking():
    return gz_to_map(MazePoint(5, 16)) # or (5, 18), or (5, 14)

def process_main(crnt, local, state):
    pass

def goal_client_init():
    goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_client.wait_for_server()
    return goal_client

def send_goal(goal_client, point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()#rospy.get_rostime()
    goal.target_pose.pose.position.x = point.x
    goal.target_pose.pose.position.y = point.y
    goal.target_pose.pose.orientation.w = 1.0

    goal_client.send_goal(goal)
    #wait = goal_client.wait_for_result(rospy.Duration.from_sec(5.0))
    #if not wait:
    #    rospy.logerr("Action server is not available!")
    #    #rospy.signal_shutdown("Action server is not available!")
    #else:
    #    print goal_client.get_result()

def send_stop_cmd(goal_client):
    goal_client.cancel_goal()

if __name__=="__main__":
    # ROS init
    rospy.init_node(NODE_NAME, log_level=rospy.INFO)
    PATH_PUB = rospy.Publisher(PATH_PUB_NAME, Path, queue_size=5)
    HARDWARE_SUB = rospy.Subscriber(HARDWARE_SUB_TOPIC, UInt8, hardware_callback, queue_size=10)
    GOAL_CLIENT = goal_client_init()
    POSE_LISTENER = tf.TransformListener()
    time.sleep(1)

    # Maze init
    crnt = get_maze_current_pose(POSE_LISTENER)
    maze = init_maze(crnt)
    #render_maze(maze)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        crnt = get_maze_current_pose(POSE_LISTENER)
        state = determinate_state(crnt)
        if state == State.MAZE_PROCESS:
            goal_point = process_maze(maze, crnt)
        elif state == State.SPEED_PROCESS:
            goal_point = process_speed()
        elif state == State.PARKING_PROCESS:
            goal_point = process_parking()
        else:
            send_stop_cmd(GOAL_CLIENT)
            continue
        send_goal(GOAL_CLIENT, goal_point)
        rospy.loginfo("%s: crnt = %s, glob_goal = %s", str(state), crnt, goal_point)
        rate.sleep()

