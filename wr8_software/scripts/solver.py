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

# Params
MAZE_TARGET_X = 1
MAZE_TARGET_Y = 2
OFFSET_X = 19
OFFSET_Y = 19
OFFSET_Z = 3.14
CELL_SZ = 2


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


class MainSolver:
    def __init__(self):
        MainSolver.hardware_status = False
        NODE_NAME = "solver_node"
        HARDWARE_SUB_TOPIC = "hardware_status"

        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
        self.HARDWARE_SUB = rospy.Subscriber(HARDWARE_SUB_TOPIC, UInt8, MainSolver._hardware_cb, queue_size=10)
        self.GOAL_CLIENT = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.GOAL_CLIENT.wait_for_server()
        self.POSE_LISTENER = tf.TransformListener()
        time.sleep(1)
        crnt = self._get_maze_current_pose()
        self.maze = MazeSolver(crnt)
        init_parking()

    def process(self):
        crnt = self._get_maze_current_pose()
        state = determinate_state(crnt)
        if state == State.MAZE_PROCESS:
            goal_point = self.maze.process(crnt)
        elif state == State.SPEED_PROCESS:
            goal_point = process_speed()
        elif state == State.PARKING_PROCESS:
            goal_point = process_parking()
        else:
            self._send_stop_cmd()
            return
        self._send_goal(goal_point)
        rospy.loginfo("%s: crnt = %s, glob_goal = %s", str(state), crnt, goal_point)

    def _get_maze_current_pose(self):
        trans = self.POSE_LISTENER.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
        #except:
        #    point = None
        return point

    def _send_goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()#rospy.get_rostime()
        goal.target_pose.pose.position.x = point.x
        goal.target_pose.pose.position.y = point.y
        goal.target_pose.pose.orientation.w = 1.0

        self.GOAL_CLIENT.send_goal(goal)
        #wait = self.GOAL_CLIENT.wait_for_result(rospy.Duration.from_sec(5.0))
        #if not wait:
        #    rospy.logerr("Action server is not available!")
        #    #rospy.signal_shutdown("Action server is not available!")
        #else:
        #    print self.GOAL_CLIENT.get_result()

    def _send_stop_cmd(self):
        self.GOAL_CLIENT.cancel_goal()

    @staticmethod
    def _hardware_cb(msg):
        rospy.logdebug("I heard hardware %s", str(msg.data))
        if msg.data == 0:
            MainSolver.hardware_status = False
        elif msg.data == 1:
            MainSolver.hardware_status = True

class MazeSolver:
    def __init__(self, initial_pose):
        MazeSolver.FRAME_ID = "map"
        PATH_PUB_NAME = "path"

        MazeSolver.PATH_PUB = rospy.Publisher(PATH_PUB_NAME, Path, queue_size=5)

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
        #edge_walls = [[MazePoint(6, 9), MazePoint(7, 9)]]

        structure = np.array(structure, np.uint8)
        self.maze = Maze(structure)#, edge_walls)
        self.maze.set_target(MazePoint(MAZE_TARGET_X, MAZE_TARGET_Y))
        self.maze.set_state(PointDir(initial_pose.x, initial_pose.y, 'D'))

        #pygame.init()
        #self.maze.render_maze()

    def process(self, crnt):
        path = self.maze.get_path()
        local = self.maze.get_local_target()
        goal_point = None
        if crnt is None:
            rospy.logerr("Maze processing error: crnt pose must exists!")
        elif local is None:
            rospy.logerr("Maze processing error: local target must exists!")
        else:
            local = local.coord
            MazeSolver._pub_path(path)
            if local.x == crnt.x and local.y == crnt.y:
                rospy.logdebug("New path:")
                for node in path:
                    rospy.logdebug("- %s", node)
                
                self.maze.next_local_target()
            goal_point = maze_to_map(path[-1].coord)
        return goal_point

    @staticmethod
    def _pub_path(nodes):
        path = Path()
        path.header.seq = 1
        path.header.frame_id = MazeSolver.FRAME_ID
        for i in range(0, len(nodes)):
            pose = PoseStamped()
            pose.header.seq = 1
            pose.header.frame_id = MazeSolver.FRAME_ID

            point = maze_to_map(nodes[i].coord)
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            rospy.logdebug("path pose is %s", str(point))
            path.poses.append(pose)
        MazeSolver.PATH_PUB.publish(path)

def init_parking():
    pass

def process_speed():
    return maze_to_map(MazePoint(1, 7))

def process_parking():
    return gz_to_map(MazePoint(5, 16)) # or (5, 18), or (5, 14)


def determinate_state(crnt):
    """
    crnt - maze current pose
    """
    state = State
    if MainSolver.hardware_status is False or crnt is None:
        state = State.IDLE
    elif crnt.x >= 3 or (crnt.x >= 2 and crnt.y <= 5):
        state = State.MAZE_PROCESS
    elif crnt.y <= 5:
        state = State.SPEED_PROCESS
    elif crnt.y >= 6:
        state = State.PARKING_PROCESS
    return state


if __name__=="__main__":
    solver = MainSolver()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        solver.process()
        rate.sleep()

