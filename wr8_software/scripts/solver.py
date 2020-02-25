#!/usr/bin/env python
import math
from enum import Enum

import rospy
import tf
import actionlib
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from car_parking.msg import Point2D, Points2D, Polygons, Statuses

import numpy as np
import pygame

from maze import MazePoint, PointDir, Maze


# Params
MAZE_TARGET_X = 1
MAZE_TARGET_Y = 2
START_X = 15
START_Y = 19
START_Z = 0 # 0 - right, 1.57 - up
CELL_SZ = 2


# Coordinate transform from MazePoint to MazePoint
def maze_to_gz(p):
    gz_x = p.x * CELL_SZ + 0.5*CELL_SZ
    gz_y = p.y * CELL_SZ + 0.5*CELL_SZ
    return MazePoint(gz_x, gz_y)

def gz_to_maze(p):
    maze_x = round((p.x - 0.5*CELL_SZ) / CELL_SZ)
    maze_y = round((p.y - 0.5*CELL_SZ) / CELL_SZ)
    return MazePoint(maze_x, maze_y)

def gz_to_map(p):
    map_x = (p.x - START_X) * math.cos(-START_Z) - (p.y - START_Y) * math.sin(-START_Z)
    map_y = (p.x - START_X) * math.sin(-START_Z) + (p.y - START_Y) * math.cos(-START_Z)
    return MazePoint(map_x, map_y)

def map_to_gz(p):
    gz_x = START_X + p.x * math.cos(START_Z) - p.y * math.sin(START_Z)
    gz_y = START_Y + p.x * math.sin(START_Z) + p.y * math.cos(START_Z)
    return MazePoint(gz_x, gz_y)

def maze_to_map(p):
    return gz_to_map(maze_to_gz(p))

def map_to_maze(p):
    return gz_to_maze(map_to_gz(p))


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
HW_SUB_TOPIC = "hardware_status"

FRAME_ID = "map"
PATH_PUB_NAME = "path"

POLY_PUB_TOPIC = "parking_polygones"
RVIZ_PUB_TOPIC = "parking_polygones_visualization_"
STATUS_SUB_TOPIC = "parking_status"
CMD_PUB_TOPIC = "parking_cmd"

PARKING_GOALS = tuple((gz_to_map(MazePoint(5, 14)),
                       gz_to_map(MazePoint(5, 16)),
                       gz_to_map(MazePoint(5, 18))))
PARKING_WAITING_GOAL = gz_to_map(MazePoint(3, 16))
PARKING_TOLERANCE = 70


class MainSolver(object):
    def __init__(self):
        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
        MainSolver.hardware_status = False
        self.hw_sub = rospy.Subscriber(HW_SUB_TOPIC, UInt8, MainSolver._hardware_cb, queue_size=10)
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.pose_listener = tf.TransformListener()
        rospy.sleep(1) # need pause before first publish but maze consructor needs crnt pose
        crnt = self._get_maze_current_pose()
        self.maze = MazeSolver(crnt)
        self.parking = ParkingSolver()

    def process(self):
        crnt = self._get_maze_current_pose()
        state = MainSolver._determinate_state(crnt)
        if state == State.MAZE_PROCESS:
            self.parking.stop_work()
            goal_point = self.maze.process(crnt)
        elif state == State.SPEED_PROCESS:
            self.parking.stop_work()
            goal_point = process_speed()
        elif state == State.PARKING_PROCESS:
            self.parking.start_work()
            goal_point = self.parking.process()
        else:
            self._send_stop_cmd()
            self.parking.stop_work()
            return
        self._send_goal(goal_point)
        self.parking.publish_to_rviz()
        rospy.loginfo("%s: crnt pose (maze) = %s, glob goal(map) = %s",
                      str(state), crnt, goal_point)

    @staticmethod
    def _determinate_state(crnt):
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
        elif crnt.y > 5:
            state = State.PARKING_PROCESS
        return state

    def _get_maze_current_pose(self):
        trans = self.pose_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
        return point

    def _send_goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = FRAME_ID
        goal.target_pose.header.stamp = rospy.Time.now()#rospy.get_rostime()
        goal.target_pose.pose.position.x = point.x
        goal.target_pose.pose.position.y = point.y
        goal.target_pose.pose.orientation.w = 1.0

        self.goal_client.send_goal(goal)
        #wait = self.goal_client.wait_for_result(rospy.Duration.from_sec(5.0))
        #if not wait:
        #    rospy.logerr("Action server is not available!")
        #    #rospy.signal_shutdown("Action server is not available!")
        #else:
        #    print self.goal_client.get_result()

    def _send_stop_cmd(self):
        self.goal_client.cancel_goal()

    @staticmethod
    def _hardware_cb(msg):
        rospy.logdebug("I heard hardware %s", str(msg.data))
        if msg.data == 0:
            MainSolver.hardware_status = False
        elif msg.data == 1:
            MainSolver.hardware_status = True

class MazeSolver(object):
    def __init__(self, initial_pose):
        MazeSolver.PATH_PUB = rospy.Publisher(PATH_PUB_NAME, Path, queue_size=5)

        structure = [[8, 8, 8, 0, 0, 0, 0, 0, 0, 0],
                     [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                     [8, 8, 8, 0, 0, 0, 0, 0, 8, 0],
                     [8, 8, 8, 0, 8, 0, 8, 0, 0, 0],
                     [8, 8, 8, 0, 8, 0, 8, 8, 8, 0],
                     [8, 8, 0, 0, 0, 0, 0, 0, 0, 0],
                     [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                     [8, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                     [8, 8, 0, 0, 0, 0, 0, 0, 0, 0]]
        edge_walls = [[MazePoint(6, 9), MazePoint(7, 9)]]

        structure = np.array(structure, np.uint8)
        self.maze = Maze(structure, edge_walls)
        self.maze.set_target(MazePoint(MAZE_TARGET_X, MAZE_TARGET_Y))
        self.maze.set_state(PointDir(initial_pose.x, initial_pose.y, 'L'))

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
        MazeSolver.PATH_PUB.publish(path)


class ParkingSolver(object):
    def __init__(self):
        rospy.Subscriber(STATUS_SUB_TOPIC, Statuses, self._status_cb, queue_size=10)
        self.pub_to_parking = rospy.Publisher(POLY_PUB_TOPIC, Polygons, queue_size=10)
        self.pub_to_cmd = rospy.Publisher(CMD_PUB_TOPIC, UInt8, queue_size=10)
        self.pub_to_rviz = list()
        for i in range(3):
            new_pub = rospy.Publisher(RVIZ_PUB_TOPIC + str(i + 1), PolygonStamped, queue_size=10)
            self.pub_to_rviz.append(new_pub)

        self.parking_polygones, self.rviz_polygones = ParkingSolver._create_polygones()
        self.fullness = list([-1, -1, -1])

    def start_work(self):
        self.pub_to_cmd.publish(1)

    def stop_work(self):
        self.pub_to_cmd.publish(0)

    def publish_to_rviz(self):
        for i in range(3):
            self.pub_to_rviz[i].publish(self.rviz_polygones[i])

    def process(self):
        self.pub_to_parking.publish(self.parking_polygones)

        goal = PARKING_WAITING_GOAL
        if len(self.fullness) == 3:
            if self.fullness[0] >= 0 and self.fullness[0] <= PARKING_TOLERANCE:
                goal = PARKING_GOALS[0]
            elif self.fullness[1] >= 0 and self.fullness[1] <= PARKING_TOLERANCE:
                goal = PARKING_GOALS[1]
            elif self.fullness[2] >= 0 and self.fullness[2] <= PARKING_TOLERANCE:
                goal = PARKING_GOALS[2]
        return goal

    @staticmethod
    def _create_polygones():
        gz_polygons = list()
        gz_polygons.append(((4.0, 12.4), (5.6, 14.4), (5.6, 15.6), (4.0, 13.6)))
        gz_polygons.append(((4.0, 14.4), (5.6, 16.4), (5.6, 17.6), (4.0, 15.6)))
        gz_polygons.append(((4.0, 16.4), (5.6, 18.4), (5.6, 19.6), (4.0, 17.6)))

        map_polygons = list()
        polygons = list()
        polygons_stamped = list()
        for i in range(0, len(gz_polygons)):
            map_polygons.append(ParkingSolver._gz_to_map_for_polygon(gz_polygons[i]))
            polygons.append(ParkingSolver._create_polygon(map_polygons[-1]))
            polygons_stamped.append(ParkingSolver._create_polygon_stamped(map_polygons[-1]))
        parking_polygones = Polygons()
        parking_polygones.polygons = tuple(polygons)
        return [parking_polygones, polygons_stamped]

    @staticmethod
    def _create_polygon(polygon):
        points2D = [Point2D(), Point2D(), Point2D(), Point2D()]
        for i in range(4):
            points2D[i].x = polygon[i][0]
            points2D[i].y = polygon[i][1]
        poly = Points2D()
        poly.points = (points2D)
        return poly

    @staticmethod
    def _create_polygon_stamped(polygon):
        points = [Point(), Point(), Point(), Point()]
        for i in range(4):
            points[i].x = polygon[i][0]
            points[i].y = polygon[i][1]
        polygons_stumped = PolygonStamped()
        polygons_stumped.header.stamp = rospy.get_rostime()
        polygons_stumped.header.frame_id = FRAME_ID
        polygons_stumped.polygon.points = tuple(points)
        return polygons_stumped

    @staticmethod
    def _gz_to_map_for_point(point):
        result = gz_to_map(MazePoint(point[0], point[1]))
        return (result.x, result.y)

    @staticmethod
    def _gz_to_map_for_polygon(polygon):
        new_polygon = tuple()
        for point in polygon:
            new_polygon += (ParkingSolver._gz_to_map_for_point(point),) # , means singleton
        return new_polygon

    def _status_cb(self, msg):
        print "I heard statuses with length {}: ".format(len(msg.fullness)),
        for i in range(0, len(msg.fullness)):
            print msg.messages[i],
            if msg.fullness[i] != -1:
                print "(" + str(msg.fullness[i]) + ") ",
        print ""
        self.fullness = msg.fullness

def process_speed():
    return maze_to_map(MazePoint(1, 7))


if __name__ == "__main__":
    solver = MainSolver()
    RATE = rospy.Rate(1)
    while not rospy.is_shutdown():
        solver.process()
        RATE.sleep()
