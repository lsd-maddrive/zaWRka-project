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
from mad_detector.msg import Detection, Detections

import numpy as np
import pygame

from maze import MazePoint, PointDir, Maze
from line_detector_ros import LineDetectorRos


# Params
MAZE_TARGET_X = 1
MAZE_TARGET_Y = 2
CELL_SZ = 2
PARKING_TOLERANCE = 70


# Coordinate transform from MazePoint to MazePoint
def maze_to_map(p):
    map_x = p.x * CELL_SZ + 0.5*CELL_SZ
    map_y = p.y * CELL_SZ + 0.5*CELL_SZ
    return MazePoint(map_x, map_y)

def map_to_maze(p):
    maze_x = round((p.x - 0.5*CELL_SZ) / CELL_SZ)
    maze_y = round((p.y - 0.5*CELL_SZ) / CELL_SZ)
    return MazePoint(maze_x, maze_y)


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

class TLColor(Enum):
    UNKNOWN = 0
    GREEN = 1
    RED = 2

class SignType(Enum):
    NO_SIGN = 0
    BRICK = 1
    ONLY_FORWARD = 2
    ONLY_RIGHT = 3
    ONLY_LEFT = 4
    FORWARD_OR_RIGHT = 5
    FORWARD_OR_LEFT = 6

SIGN_TYPE_TO_MAZE = tuple(([0, 0, 0], [0, 1, 0], [1, 0, 1], [1, 1, 0],
                           [0, 1, 1], [1, 0, 0], [0, 0, 1]))

NODE_NAME = "solver_node"
HW_SUB_TOPIC = "hardware_status"
SIGN_SUB_TOPIC = "detected_sign_type"
TL_SUB_TOPIC = "tl_status"
WL_SUB_TOPIC = "wl_status"
PATH_PUB_TOPIC = "path"

FRAME_ID = "map"

POLY_PUB_TOPIC = "parking_polygones"
RVIZ_PUB_TOPIC = "parking_polygones_visualization_"
STATUS_SUB_TOPIC = "parking_status"
CMD_PUB_TOPIC = "parking_cmd"


class MainSolver(object):
    def __init__(self):
        rospy.init_node(NODE_NAME, log_level=rospy.INFO)
        MainSolver._init_params()
        self.hardware_status = False
        self.previous_goal = None
        rospy.Subscriber(HW_SUB_TOPIC, UInt8, self._hardware_cb, queue_size=10)
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.pose_listener = tf.TransformListener()
        rospy.sleep(1) # need pause before first publish but maze consructor needs crnt pose
        self.maze = MazeSolver(self._get_maze_current_pose())
        self.parking = ParkingSolver()

    def process(self):
        """
        Determinate stage and state, then send goal or stop cmd to move_base.
        """
        maze_crnt_pose = self._get_maze_current_pose()
        state = self._determinate_stage(maze_crnt_pose)
        goal_point = None
        map_direction = 0

        if state == State.MAZE_PROCESS or state == State.MAZE_WAIT_TL or \
           state == State.MAZE_WAIT_SIGNS or state == State.MAZE_TARGET_REACHED:
            self.parking.stop_work()
            self.maze.start_async_processing()
            if self.maze.is_recovery_need == True:
                self.maze.reinit(maze_crnt_pose)
                self.maze.is_recovery_need = False
            goal_point, map_direction = self.maze.process(maze_crnt_pose)
            if goal_point is None:
                state = State.MAZE_WAIT_TL
        elif state == State.SPEED_PROCESS or state == State.SPEED_REACHED:
            self.parking.stop_work()
            self.maze.stop_async_processing()
            goal_point, map_direction = process_speed()
        elif state == State.PARKING_PROCESS or state == State.PARKING_REACHED:
            self.parking.start_work()
            self.maze.stop_async_processing()
            goal_point, map_direction = self.parking.process()

        if goal_point is None:
            self._send_stop_cmd()
            self.parking.stop_work()
            rospy.loginfo("%s: from %s to None", str(state), maze_to_map(maze_crnt_pose))
        else:
            self._send_goal(goal_point, map_direction)
            rospy.loginfo("%s: from %s to %s",
                          str(state), maze_to_map(maze_crnt_pose), goal_point)
        self.parking.publish_to_rviz()
        self.previous_goal = goal_point

    @staticmethod
    def _init_params():
        global MAZE_TARGET_X, MAZE_TARGET_y, CELL_SZ, PARKING_TOLERANCE
        MAZE_TARGET_X = rospy.get_param('~maze_target_x', 1)
        MAZE_TARGET_Y = rospy.get_param('~maze_target_y', 2)
        CELL_SZ = rospy.get_param('~cell_sz', 2)
        PARKING_TOLERANCE = rospy.get_param('~parking_tolerance', 70)

    def _determinate_stage(self, maze_crnt_pose):
        state = State.IDLE
        if self.hardware_status is False or maze_crnt_pose is None:
            state = State.IDLE
        elif maze_crnt_pose.x >= 3 or (maze_crnt_pose.x >= 2 and maze_crnt_pose.y <= 5):
            state = State.MAZE_PROCESS
        elif maze_crnt_pose.y <= 5:
            state = State.SPEED_PROCESS
        elif maze_crnt_pose.y > 5:
            state = State.PARKING_PROCESS
        return state

    def _get_maze_current_pose(self):
        trans = self.pose_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
        return point

    def _send_goal(self, map_point, map_angle = 0):
        """
        map_angle: 0 - right, 1.57 - up
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = FRAME_ID
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = map_point.x
        goal.target_pose.pose.position.y = map_point.y
        goal.target_pose.pose.orientation.z = math.sin(map_angle / 2)
        goal.target_pose.pose.orientation.w = math.cos(map_angle / 2)
        self.goal_client.send_goal(goal)

    def _send_stop_cmd(self):
        if self.previous_goal != None: 
            self.goal_client.cancel_goal()

    def _hardware_cb(self, msg):
        rospy.logdebug("I heard hardware %s", str(msg.data))
        if msg.data == 0:
            self.hardware_status = False
        elif msg.data == 1:
            rospy.Timer(rospy.Duration(5), self._change_hw_status_cb, oneshot=True)

    def _change_hw_status_cb(self, event):
        rospy.loginfo("Solver was started!")
        self.hardware_status = True


class MazeSolver(object):
    STRUCTURE = np.array([[8, 8, 8, 0, 0, 0, 0, 0, 0, 0],
                          [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                          [8, 8, 8, 0, 0, 0, 0, 0, 8, 0],
                          [8, 8, 8, 0, 8, 0, 8, 0, 0, 0],
                          [8, 8, 8, 0, 8, 0, 8, 8, 8, 0],
                          [8, 8, 0, 0, 0, 0, 0, 0, 0, 0],
                          [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                          [8, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [8, 8, 8, 0, 8, 8, 0, 8, 8, 0],
                          [8, 8, 0, 0, 0, 0, 0, 0, 0, 0]], np.uint8)
    EDGE_WALLS = [[MazePoint(6, 9), MazePoint(7, 9)]]

    def __init__(self, maze_initial_pose):
        self.wl_msg = UInt8()
        self.is_wl_appeared = False
        self.tl_msg = Detections()
        self.tl_color = TLColor.UNKNOWN
        self.sign_msg = UInt8()
        self.sign_type = SignType.NO_SIGN
        self.is_recovery_need = False
        self.line_detector = LineDetectorRos(math.pi*7/16, math.pi*9/16, 10, "stereo_camera_converted/left/image_raw", "img_from_core")

        self.PATH_PUB = rospy.Publisher(PATH_PUB_TOPIC, Path, queue_size=5)
        rospy.Subscriber(SIGN_SUB_TOPIC, UInt8, self._sign_cb, queue_size=10)
        rospy.Subscriber(TL_SUB_TOPIC, Detections, self._tl_cb, queue_size=10)
        rospy.Subscriber(WL_SUB_TOPIC, UInt8, self._wl_cb, queue_size=10)

        self.reinit(maze_initial_pose)

        #pygame.init()
        #self.maze.render_maze()

    def reinit(self, maze_initial_pose):
        self.maze = Maze(MazeSolver.STRUCTURE, MazeSolver.EDGE_WALLS)
        self.maze.set_target(MazePoint(MAZE_TARGET_X, MAZE_TARGET_Y))
        self.maze.set_state(PointDir(maze_initial_pose.x, maze_initial_pose.y, 'L'))

    def start_async_processing(self):
        self.line_detector.enable()

    def stop_async_processing(self):
        self.line_detector.disable()

    def process(self, maze_crnt_pose):
        """ Calcaulate goal_point (in map format) and update path if necessary
        - if success: return actual goal point and pub path
        - if error: return default goal_point
        - if white line: return None that means stop
        """
        DEFAULT_GOAL_POINT = MazePoint(MAZE_TARGET_X, MAZE_TARGET_Y)
        WHITE_LINE_GOAL_POINT = None
        MAP_DIRECTION = 1.57

        try:
            path = self.maze.get_path()
        except:
            rospy.logerr("Maze processing error: path can't be empty!")
            self.is_recovery_need = True
            return DEFAULT_GOAL_POINT, MAP_DIRECTION

        local = self.maze.get_local_target()

        # Process errors in old path
        if maze_crnt_pose is None:
            rospy.logerr("Maze processing error: maze crnt pose must exists!")
            self.is_recovery_need = True
            return DEFAULT_GOAL_POINT, MAP_DIRECTION
        elif local is None:
            rospy.logerr("Maze processing error: local target must exists!")
            self.is_recovery_need = True
            return DEFAULT_GOAL_POINT, MAP_DIRECTION

        # Process white line and traffic light
        self._update_status_of_white_line()
        self._update_status_of_crnt_tl()
        if self.is_wl_appeared == True:
            if self.tl_color == TLColor.GREEN:
                rospy.loginfo("There is WL and TL is green: way is clear.")
                self.is_wl_appeared = False
            elif self.tl_color == TLColor.UNKNOWN:
                rospy.logwarn("There is WL but TL is unknown: is it error?")
                self.is_wl_appeared = False
            else:
                rospy.loginfo_throttle(2, "There is WL and TL is red: wait...")
                return WHITE_LINE_GOAL_POINT, MAP_DIRECTION

        # Update next local target and pub new path if possible
        if local.coord == maze_crnt_pose:
            if self.maze.next_local_target() == False:
                rospy.logerr("Current path length is 0, can't get next tartet!")
                self.is_recovery_need = True
                return DEFAULT_GOAL_POINT, MAP_DIRECTION
            if len(path) == 0:
                rospy.logerr("Path is empty!")
                self.is_recovery_need = True
                return DEFAULT_GOAL_POINT, MAP_DIRECTION
        
        # Process signs
        self._update_status_of_crnt_sign()
        if self.sign_type != SignType.NO_SIGN:
            rospy.loginfo("A sign was detected: %s", str(self.sign_type))
            self.maze.set_limitation(SIGN_TYPE_TO_MAZE[self.sign_type.value])
            try:
                path = self.maze.get_path()
            except:
                rospy.logerr("Maze processing error: path is empty!")
                self.is_recovery_need = True
                return DEFAULT_GOAL_POINT, MAP_DIRECTION

        # Return actual goal point
        self._pub_path(path)
        goal_point = maze_to_map(path[-1].coord)
        return goal_point, MAP_DIRECTION

    def _update_status_of_crnt_tl(self):
        previous_tl_color = self.tl_color

        if len(self.tl_msg.detections) == 0:
            self.tl_color = TLColor.UNKNOWN
        elif self.tl_msg.detections[0].object_class == "traffic_light_red":
            self.tl_color = TLColor.RED
        elif self.tl_msg.detections[0].object_class == "traffic_light_green":
            self.tl_color = TLColor.GREEN
        else:
            self.tl_color = TLColor.UNKNOWN

        if previous_tl_color != self.tl_color:
            rospy.loginfo("A TL was status has been changed to %s", str(self.tl_color))

    def _update_status_of_crnt_sign(self):
        previous_sign_type = self.sign_type

        if self.sign_msg is None:
            self.sign_type = SignType.UNKNOWN
        else:
            self.sign_type = SignType(self.sign_msg.data)

        if previous_sign_type != self.sign_type:
            rospy.loginfo("A sign status has been changed to %s", str(self.sign_type))

    def _update_status_of_white_line(self):
        previous_wl_status = self.is_wl_appeared

        result = self.line_detector.get_lines()
        if result is not None:
            self.is_wl_appeared = True if len(result) >= 2 else False

        if previous_wl_status != self.is_wl_appeared:
            rospy.loginfo("A white line status has been changed to %d", self.wl_msg.data)

    def _pub_path(self, nodes):
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
            path.poses.append(pose)
        self.PATH_PUB.publish(path)

    def _sign_cb(self, msg):
        self.sign_msg = msg

    def _tl_cb(self, msg):
        self.tl_msg = msg

    def _wl_cb(self, msg):
        self.wl_msg = msg


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

        self.PARKING_GOALS = tuple((MazePoint(5, 14),
                                    MazePoint(5, 16),
                                    MazePoint(5, 18)))
        self.PARKING_WAITING_GOAL = MazePoint(3, 16)

    def start_work(self):
        self.pub_to_cmd.publish(1)

    def stop_work(self):
        self.pub_to_cmd.publish(0)

    def publish_to_rviz(self):
        for i in range(3):
            polygon = self.rviz_polygones[i]
            self.pub_to_rviz[i].publish(polygon)

    def process(self):
        self.pub_to_parking.publish(self.parking_polygones)

        goal = self.PARKING_WAITING_GOAL
        map_direction = 1.57
        if len(self.fullness) == 3:
            if self.fullness[0] >= 0 and self.fullness[0] <= PARKING_TOLERANCE:
                goal = self.PARKING_GOALS[0]
                map_direction = 0.785
            elif self.fullness[1] >= 0 and self.fullness[1] <= PARKING_TOLERANCE:
                goal = self.PARKING_GOALS[1]
                map_direction = 0.785
            elif self.fullness[2] >= 0 and self.fullness[2] <= PARKING_TOLERANCE:
                goal = self.PARKING_GOALS[2]
                map_direction = 0.785
        else:
            rospy.logwarn("Statuses length is not correct!")
        return goal, map_direction

    @staticmethod
    def _create_polygones():
        list_polygons = list()
        list_polygons.append(((4.0, 12.4), (5.6, 14.4), (5.6, 15.6), (4.0, 13.6)))
        list_polygons.append(((4.0, 14.4), (5.6, 16.4), (5.6, 17.6), (4.0, 15.6)))
        list_polygons.append(((4.0, 16.4), (5.6, 18.4), (5.6, 19.6), (4.0, 17.6)))

        map_polygons = list()
        polygons = list()
        polygons_stamped = list()
        for i in range(0, len(list_polygons)):
            map_polygons.append(ParkingSolver._create_map_polygon(list_polygons[i]))
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
    def _create_map_polygon(polygon):
        new_polygon = tuple()
        for point in polygon:
            new_polygon += ((point[0], point[1]),) # , means singleton
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
    return maze_to_map(MazePoint(1, 7)), 1.57


if __name__ == "__main__":
    solver = MainSolver()
    RATE = rospy.Rate(3)
    while not rospy.is_shutdown():
        solver.process()
        RATE.sleep()
