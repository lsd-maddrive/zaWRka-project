#!/usr/bin/env python2
"""
    This node subscribes on START_BTN_TOPIC.
    If start button is pressed, it will determinate one of 3 possible stage
and publish a goal using move_base action.
    For each stages, this node do smth specific things:
    1. Maze: subscribes on SIGN_SUB_TOPIC and WL_SUB_TOPIC to get info from
detectors and publish path to waypoint global planner.
    2. Speed stage: do nothing specific things.
    3. Parking stage: subscribes on STATUS_SUB_TOPIC with info about free
parking place and determinate goal on free place.
"""

import math
from enum import Enum
from copy import copy

import rospy
import tf
import actionlib
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from car_parking.msg import Point2D, Points2D, Polygons, Statuses
from mad_detector.msg import Detections
from std_srvs.srv import Empty

import numpy as np

from maze import MazePoint, PointDir, Maze
from line_detector_ros import LineDetectorRos
from image_utils import ImageTopicReceiver, CompressedImageTopicPublisher, ImageTopicPublisher
from signs_detector import SignDetector
from feature_map import FeatureHandler, TLColor, SignType

# Params
CELL_SZ = 2
FRAME_ID = "map"

# Constants
SIGN_TYPE_TO_MAZE = tuple(([0, 0, 0], [0, 1, 0], [1, 0, 1], [1, 1, 0],
                           [0, 1, 1], [1, 0, 0], [0, 0, 1]))

class PrintColor:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

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

# Coordinate transform from MazePoint to MazePoint
def maze_to_map(p):
    map_x = p.x * CELL_SZ + 0.5*CELL_SZ
    map_y = p.y * CELL_SZ + 0.5*CELL_SZ
    return MazePoint(map_x, map_y)

def map_to_maze(p):
    maze_x = round((p.x - 0.5*CELL_SZ) / CELL_SZ)
    maze_y = round((p.y - 0.5*CELL_SZ) / CELL_SZ)
    return MazePoint(maze_x, maze_y)

class Maze2020Version1:
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

    @staticmethod
    def get_stage(maze_crnt_pose):
        stage = State.IDLE
        if maze_crnt_pose.x >= 3 or (maze_crnt_pose.x >= 2 and maze_crnt_pose.y <= 5):
            stage = State.MAZE_PROCESS
        elif maze_crnt_pose.y <= 5:
            stage = State.SPEED_PROCESS
        elif maze_crnt_pose.y > 5:
            stage = State.PARKING_PROCESS
        return stage

class Maze2020Version2:
    STRUCTURE = np.array([[8, 8, 0, 8, 8, 8, 8, 8, 8, 8],
                         [8, 8, 0, 0, 8, 8, 8, 8, 8, 8],
                         [0, 8, 0, 8, 0, 8, 8, 8, 8, 8],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [8, 8, 8, 0, 8, 0, 8, 8, 0, 8],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [8, 0, 8, 0, 8, 0, 8, 8, 8, 8],
                         [8, 0, 8, 0, 8, 0, 8, 8, 8, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], np.uint8)
    EDGE_WALLS = []
    WORLD_X = 10
    WORLD_Y = 9
    @staticmethod
    def get_stage(maze_crnt_pose):
        stage = State.IDLE
        if maze_crnt_pose.x < 0 or maze_crnt_pose.x > Maze2020Version2.WORLD_X or \
           maze_crnt_pose.y < 0 or maze_crnt_pose.y > Maze2020Version2.WORLD_Y:
            stage = State.IDLE
        if maze_crnt_pose.y < 6 or (maze_crnt_pose.y < 7 and maze_crnt_pose.x < 6):
            stage = State.MAZE_PROCESS
        elif maze_crnt_pose.y >= 7 and maze_crnt_pose.x <= 4:
            stage = State.SPEED_PROCESS
        elif maze_crnt_pose.y == 6 and maze_crnt_pose.x >= 6:
            stage = State.PARKING_REACHED
        elif maze_crnt_pose.y >= 6 and maze_crnt_pose.x >= 4:
            stage = State.PARKING_PROCESS
        return stage

VERSION_TO_MAZE = {1 : Maze2020Version1, 2 : Maze2020Version2}

class MainSolver(object):
    def __init__(self):
        rospy.init_node("solver_node", log_level=rospy.INFO)
        self.BUTTON_TOPIC = rospy.get_param('~button_topic', "cmd")

        global CELL_SZ, FRAME_ID
        CELL_SZ = rospy.get_param('~cell_sz', 2)
        FRAME_ID = rospy.get_param('~frame_id', 'map')
        self.SPEED_GOAL_POSE = MazePoint(*rospy.get_param('~speed_goal_pose'))
        self.SPEED_GOAL_ORIENTATION = rospy.get_param('~speed_goal_orientation')
        self.world = VERSION_TO_MAZE[rospy.get_param('~maze_version', 2)]

        self.prev_stage = State.IDLE
        self.hardware_status = False
        self.start_time = rospy.get_rostime()
        self.previous_goal = None
        rospy.Subscriber(self.BUTTON_TOPIC, String, self._button_cb, queue_size=10)
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.pose_listener = tf.TransformListener()
        rospy.sleep(1) # need pause before first publish but maze consructor needs crnt pose
        map_pose, orientation = self._get_map_current_pose()
        self.maze = MazeSolver(map_pose, orientation)
        self.parking = ParkingSolver()

    def process(self):
        """
        Determinate stage and state, then send goal or stop cmd to move_base.
        """
        map_crnt_pose, orientation = self._get_map_current_pose()
        state = self._determinate_stage(map_to_maze(map_crnt_pose))
        self.goal_point = None
        map_direction = 0

        # prevent changes from maze to parking and from parking to maze
        if self.prev_stage == State.MAZE_PROCESS and state == State.PARKING_PROCESS:
            state = self.prev_stage
        elif self.prev_stage == State.PARKING_PROCESS and state == State.MAZE_PROCESS:
            state = self.prev_stage
        elif self.prev_stage != State.PARKING_PROCESS and state == State.PARKING_REACHED:
            state = self.prev_stage
        elif self.prev_stage == State.PARKING_REACHED:
            state = self.prev_stage
        self.prev_stage = state

        if state == State.MAZE_PROCESS or state == State.MAZE_WAIT_TL or \
           state == State.MAZE_WAIT_SIGNS or state == State.MAZE_TARGET_REACHED:
            self.parking.stop()
            self.maze.start()
            if self.maze.is_recovery_need:
                self.maze.reinit(map_crnt_pose, orientation)
                self.maze.is_recovery_need = False
            self.goal_point, map_direction = self.maze.process(map_crnt_pose, orientation)
            if self.goal_point is None:
                state = State.MAZE_WAIT_TL
        elif state == State.SPEED_PROCESS or state == State.SPEED_REACHED:
            self.parking.stop()
            self.maze.stop()
            self.goal_point = self.SPEED_GOAL_POSE
            map_direction = self.SPEED_GOAL_ORIENTATION
        elif state == State.PARKING_PROCESS:
            self.parking.start()
            self.maze.stop()
            self.goal_point, map_direction = self.parking.process()
        elif state == State.PARKING_REACHED:
            self.parking.stop()
            self.maze.stop()
            self.goal_point = None

        self.map_x_pose = map_crnt_pose.x
        self.map_y_pose = map_crnt_pose.y
        self.orientation = orientation
        if self.goal_point is None:
            self._send_stop_cmd()
            self.parking.stop()
        else:
            self._send_goal(self.goal_point, map_direction)
        self.parking.publish_to_rviz()
        self.previous_goal = self.goal_point
        self._log(state)

    def _log(self, state):
        if state == State.IDLE:
            state_print_color = PrintColor.HEADER
        elif state == State.MAZE_PROCESS:
            state_print_color = PrintColor.OKGREEN
        elif state == State.MAZE_WAIT_TL:
            state_print_color = PrintColor.FAIL
        elif state == State.SPEED_PROCESS:
            state_print_color = PrintColor.OKBLUE
        elif state == State.PARKING_PROCESS:
            state_print_color = PrintColor.WARNING
        else:
            state_print_color = PrintColor.UNDERLINE
        state_info = state_print_color + str(state) + PrintColor.ENDC

        rospy.loginfo("%s: [%.1f, %.1f, %.1f] -> %s, Maze=[%s]",
                      state_info,
                      self.map_x_pose,
                      self.map_y_pose,
                      self.orientation,
                      self.goal_point,
                      self.maze._get_log())

    def _determinate_stage(self, maze_crnt_pose):
        state = State.IDLE
        if self.hardware_status is True and rospy.get_rostime().secs > self.start_time.secs + 300:
            rospy.loginfo("Time is over. Stop the robot!")
            state = State.IDLE
        elif self.hardware_status is False or maze_crnt_pose is None:
            state = State.IDLE
        else:
            state = self.world.get_stage(maze_crnt_pose)
        return state

    def _get_map_current_pose(self):
        trans = self.pose_listener.lookupTransform(FRAME_ID, 'base_footprint', rospy.Time(0))
        point = MazePoint(trans[0][0], trans[0][1])
        w = trans[1][2]
        z = trans[1][3]
        t3 = +2.0 * (w * z)
        t4 = +1.0 - 2.0 * (z * z)
        yaw = math.atan2(t3, t4)
        map_yaw = 3.14 - yaw
        return point, map_yaw

    def _send_goal(self, map_point, map_angle=0):
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

    def _button_cb(self, msg):
        rospy.logdebug("I heard button %s", str(msg.data))
        if msg.data.lower() == 'stop':
            self.hardware_status = False
        elif msg.data.lower() == 'start':
            rospy.Timer(rospy.Duration(5), self._change_hw_status_cb, oneshot=True)

    def _change_hw_status_cb(self, event):
        rospy.wait_for_service('/move_base/clear_costmaps', 2)
        try:
            costmap_clear_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            costmap_clear_service()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        rospy.loginfo("Solver was started!")
        self.hardware_status = True
        self.start_time = rospy.get_rostime()


class MazeSolver(object):
    def __init__(self, initial_map_pose, orientation):
        self.wl_msg = UInt8()
        self.detections = None
        # hack: we always think, that wl is appeared, because in competetion camera is very bad!
        # normally, is_wl_appeared should be initially False
        self.is_wl_appeared = True
        self.tl_color = TLColor.UNKNOWN
        self.sign_type = SignType.NO_SIGN
        self.is_recovery_need = False

        self.MAZE_TARGET_X = rospy.get_param('~maze_target_x', 1)
        self.MAZE_TARGET_Y = rospy.get_param('~maze_target_y', 2)
        self.MAZE_TARGET_YAW = rospy.get_param('~maze_target_yaw', 1.57)
        self.WORLD = VERSION_TO_MAZE[rospy.get_param('~maze_version', 2)]

        self.WL_SUB_TOPIC = rospy.get_param('~white_line_detector/status_topic')
        self.SIGN_SUB_TOPIC = rospy.get_param('~detected_sign_type_topic')
        self.PATH_PUB_TOPIC = "path"

        RENDER_MAZE = rospy.get_param('~render_maze', False)

        # TODO - better publish with Compressed but now it doesn`t work =(
        self.line_detector = LineDetectorRos(
            receiver=ImageTopicReceiver(
                topic=rospy.get_param('~white_line_detector/camera_topic'),
                hFlip=False, vFlip=False, queue_size=10
            ),
            publisher=ImageTopicPublisher(
                topic=rospy.get_param('~white_line_detector/img_topic', None),
                queue_size=5
            )
        )
        
        self.sign_detector = SignDetector(
            frequency=rospy.get_param('~mad_detector/frequency'), 
            receiver=ImageTopicReceiver(
                topic=rospy.get_param('~mad_detector/signs_camera'),
                hFlip=True, vFlip=True, queue_size=10
            ),
            publisher=ImageTopicPublisher(
                topic=rospy.get_param('~mad_detector/signs_image', None),
                queue_size=5
            )
        )

        self.PATH_PUB = rospy.Publisher(self.PATH_PUB_TOPIC, Path, queue_size=5)
        rospy.Subscriber(self.SIGN_SUB_TOPIC, Detections, self._sign_cb, queue_size=10)
        rospy.Subscriber(self.WL_SUB_TOPIC, UInt8, self._wl_cb, queue_size=10)

        self.reinit(initial_map_pose, orientation)

        if RENDER_MAZE is True:
            import pygame
            pygame.init()
            self.maze.render_maze()

        self.feature_map = FeatureHandler(self.WORLD.STRUCTURE)

    def _get_log(self):
        LOG_COLOR = {
            TLColor.GREEN               : PrintColor.OKGREEN,
            TLColor.RED                 : PrintColor.FAIL,

            SignType.BRICK              : PrintColor.OKBLUE,
            SignType.ONLY_FORWARD       : PrintColor.OKBLUE,
            SignType.ONLY_RIGHT         : PrintColor.OKBLUE,
            SignType.ONLY_LEFT          : PrintColor.OKBLUE,
            SignType.FORWARD_OR_RIGHT   : PrintColor.OKBLUE,
            SignType.FORWARD_OR_LEFT    : PrintColor.OKBLUE,

            True                        : PrintColor.FAIL,
            False                       : PrintColor.HEADER
        }
        END = PrintColor.HEADER

        tl_print_color = LOG_COLOR[self.tl_color] if self.tl_color in LOG_COLOR else END
        sign_print_color = LOG_COLOR[self.sign_type] if self.sign_type in LOG_COLOR else END
        wl_print_color = LOG_COLOR[self.is_wl_appeared] if self.is_wl_appeared in LOG_COLOR else END

        tl_info = tl_print_color + str(self.tl_color) + PrintColor.ENDC
        sign_info = sign_print_color + str(self.sign_type) + PrintColor.ENDC
        wl_info = wl_print_color + "WL." + str(self.is_wl_appeared) + PrintColor.ENDC

        if self.tl_color == TLColor.UNKNOWN and self.is_wl_appeared == True:
            rospy.logwarn("There is WL but TL is unknown: is it error?")

        return tl_info + ", " + sign_info + ", " + wl_info

    def reinit(self, initial_map_pose, orientation):
        initial_maze_pose = map_to_maze(initial_map_pose)

        # hack: maze.py have strange bug with started orientation
        # todo: fix
        if orientation < 0:
            orientation += 6.28
        if orientation < np.pi/4 or orientation > 7*np.pi/4:
            orientation = 'L' # good for version 1 (but actually robot looks RIGHT)
        elif orientation < 3*np.pi/4:
            orientation = 'D' # ?
        elif orientation < 5*np.pi/4:
            orientation = 'U' # ?
        else:
            orientation = 'U' # good for version 2 (but actually robot looks DOWN)

        self.maze = Maze(self.WORLD.STRUCTURE, self.WORLD.EDGE_WALLS)
        self.maze.set_target(map_to_maze(MazePoint(self.MAZE_TARGET_X, self.MAZE_TARGET_Y)))
        self.maze.set_state(PointDir(initial_maze_pose.x, initial_maze_pose.y, orientation))

    def start(self):
        self.line_detector.start()
        self.sign_detector.start()

    def stop(self):
        self.line_detector.stop()
        self.sign_detector.stop()

    def process(self, map_crnt_pose, orientation):
        """ Calcaulate goal_point (in map format) and update path if necessary
        - if success: return actual goal point and pub path
        - if error: return default goal_point
        - if white line: return None that means stop
        """
        DEFAULT_GOAL_POINT = MazePoint(self.MAZE_TARGET_X, self.MAZE_TARGET_Y)
        WHITE_LINE_GOAL_POINT = None
        MAP_DIRECTION = self.MAZE_TARGET_YAW
        maze_crnt_pose = map_to_maze(map_crnt_pose)

        # 1. Check current situation
        try:
            path = self.maze.get_path()
        except:
            rospy.logerr("Maze processing error: path can't be empty!")
            self.is_recovery_need = True
            return DEFAULT_GOAL_POINT, MAP_DIRECTION
        local = self.maze.get_local_target()

        # 2. Check errors in old path
        if maze_crnt_pose is None:
            rospy.logerr("Maze processing error: maze crnt pose must exists!")
            self.is_recovery_need = True
            return DEFAULT_GOAL_POINT, MAP_DIRECTION
        elif local is None:
            rospy.logerr("Maze processing error: local target must exists!")
            self.is_recovery_need = True
            return DEFAULT_GOAL_POINT, MAP_DIRECTION

        # 3. Collect all detectors results
        self.feature_map.update(self.detections, maze_crnt_pose, orientation)
        self.sign_type = self.feature_map.get_sign_status(maze_crnt_pose, orientation)
        self.tl_color = self.feature_map.get_tl_status(maze_crnt_pose, orientation)
        self.is_wl_appeared = self._update_status_of_white_line()

        # 4. Stop is there are white line and TL is red
        if self.is_wl_appeared is True and self.tl_color == TLColor.RED:
            return WHITE_LINE_GOAL_POINT, MAP_DIRECTION

        # 5. Update path if local target node has beed arrived
        if local.coord == maze_crnt_pose:
            if self.maze.next_local_target() is False:
                rospy.logerr("Current path length is 0, can't get next tartet!")
                self.is_recovery_need = True
                return DEFAULT_GOAL_POINT, MAP_DIRECTION
            if len(path) == 0:
                rospy.logerr("Path is empty!")
                self.is_recovery_need = True
                return DEFAULT_GOAL_POINT, MAP_DIRECTION
            rospy.logdebug("Path has been updated because robot has reached local goal!")

        # 6. Process signs
        if self.sign_type != SignType.NO_SIGN:
            self.maze.set_limitation(SIGN_TYPE_TO_MAZE[self.sign_type.value])
            try:
                path = self.maze.get_path()
            except:
                rospy.logerr("Maze processing error: path is empty!")
                self.is_recovery_need = True
                return DEFAULT_GOAL_POINT, MAP_DIRECTION
            rospy.logdebug("Path has been updated because of sign!")

        # 7. Return actual goal point
        self._pub_path(path)
        goal_point = maze_to_map(path[-1].coord)
        return goal_point, MAP_DIRECTION

    def _update_status_of_white_line(self):
        # hack: we always think, that wl is appeared, because in competetion camera is very bad!
        # normally, is_wl_appeared should be initially False
        is_wl_appeared = True
        result = self.line_detector.get_lines()
        if result is not None and len(result) >= 2:
            is_wl_appeared = True
        return is_wl_appeared

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
        self.detections = copy(msg.detections)

    def _wl_cb(self, msg):
        self.wl_msg = msg


class ParkingSolver(object):
    def __init__(self):
        self.PARKING_TOLERANCE = rospy.get_param('~parking_tolerance', 70)
        self.POLY_PUB_TOPIC = "parking_polygones"
        RVIZ_PUB_TOPIC = "parking_polygones_visualization_"
        STATUS_SUB_TOPIC = "parking_status"
        CMD_PUB_TOPIC = "parking_cmd"
        self.PARKING_AREAS = rospy.get_param('~parking_areas')
        self.PARKING_WAITING_GOAL_POSE = MazePoint(*rospy.get_param('~parking_waiting_goal_pose'))
        self.PARKING_WAITING_GOAL_DIR = rospy.get_param('~parking_waiting_goal_dir')
        self.PARKING_GOAL_DIR = rospy.get_param('~parking_goal_dir')

        PARKING_GOALS = [None] * len(self.PARKING_AREAS)
        for idx in range(len(self.PARKING_AREAS)):
            PARKING_GOALS[idx] = [sum([pose[0] for pose in self.PARKING_AREAS[idx]]) / 4,
                                  sum([pose[1] for pose in self.PARKING_AREAS[idx]]) / 4]

        rospy.Subscriber(STATUS_SUB_TOPIC, Statuses, self._status_cb, queue_size=10)
        self.pub_to_parking = rospy.Publisher(self.POLY_PUB_TOPIC, Polygons, queue_size=10)
        self.pub_to_cmd = rospy.Publisher(CMD_PUB_TOPIC, UInt8, queue_size=10)
        self.pub_to_rviz = list()
        for i in range(3):
            new_pub = rospy.Publisher(RVIZ_PUB_TOPIC + str(i + 1), PolygonStamped, queue_size=10)
            self.pub_to_rviz.append(new_pub)

        self.parking_polygones, self.rviz_polygones = self._create_polygones()
        self.fullness = [-1] * len(self.PARKING_AREAS)

        self.PARKING_GOALS = tuple((MazePoint(*PARKING_GOALS[0]),
                                    MazePoint(*PARKING_GOALS[1]),
                                    MazePoint(*PARKING_GOALS[2])))

    def start(self):
        self.pub_to_cmd.publish(1)

    def stop(self):
        self.pub_to_cmd.publish(0)

    def publish_to_rviz(self):
        for i in range(3):
            polygon = self.rviz_polygones[i]
            self.pub_to_rviz[i].publish(polygon)

    def process(self):
        self.pub_to_parking.publish(self.parking_polygones)

        goal = self.PARKING_WAITING_GOAL_POSE
        map_direction = self.PARKING_WAITING_GOAL_DIR
        if len(self.fullness) == 3:
            if self.fullness[0] >= 0 and self.fullness[0] <= self.PARKING_TOLERANCE:
                goal = self.PARKING_GOALS[0]
                map_direction = self.PARKING_GOAL_DIR
            elif self.fullness[1] >= 0 and self.fullness[1] <= self.PARKING_TOLERANCE:
                goal = self.PARKING_GOALS[1]
                map_direction = self.PARKING_GOAL_DIR
            elif self.fullness[2] >= 0 and self.fullness[2] <= self.PARKING_TOLERANCE:
                goal = self.PARKING_GOALS[2]
                map_direction = self.PARKING_GOAL_DIR
        else:
            rospy.logwarn("Statuses length is not correct!")
        return goal, map_direction

    def _create_polygones(self):
        map_polygons = list()
        polygons = list()
        polygons_stamped = list()
        for i in range(0, len(self.PARKING_AREAS)):
            map_polygons.append(ParkingSolver._create_map_polygon(self.PARKING_AREAS[i]))
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
        print("I heard statuses with length {}: ".format(len(msg.fullness)))
        for i in range(0, len(msg.fullness)):
            print(msg.messages[i])
            if msg.fullness[i] != -1:
                print("(" + str(msg.fullness[i]) + ") ")
        print("")
        self.fullness = msg.fullness

if __name__ == "__main__":
    solver = MainSolver()
    RATE = rospy.Rate(rospy.get_param('~solver/frequency'))
    while not rospy.is_shutdown():
        solver.process()
        RATE.sleep()
