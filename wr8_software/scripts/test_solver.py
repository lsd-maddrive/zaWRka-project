#!/usr/bin/env python2
from time import sleep
from enum import Enum
import math

import rospy
import tf
from std_msgs.msg import UInt8
from mad_detector.msg import Detection, Detections

from maze import MazePoint
from solver import map_to_maze
from feature_map import TL_COLOR_TO_STR, SIGN_TYPE_TO_STR, TLColor, SignType


def get_cur_maze_pose(pose_listener):
    try:
        trans = pose_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
        w = trans[1][2]
        z = trans[1][3]
        t3 = +2.0 * (w * z)
        t4 = +1.0 - 2.0 * (z * z)
        yaw = math.atan2(t3, t4)
        map_yaw = 3.14 - yaw
    except tf.LookupException as err:
        rospy.logwarn("tf2.LookupException: %s", err)
        point = None
        map_yaw = None
    except Exception as err:
        rospy.logwarn(":( %s", err)
        point = None
        map_yaw = None
    return point, map_yaw

class AbstractRoadObject(object):
    def __init__(self):
        self.object_type = None

    def process(self):
        self.object_type = self.do()

    def get_type(self):
        if self.object_type is None:
            return "no object"
        return self.object_type

    def do(self):
        return None


class Sign(AbstractRoadObject):
    def __init__(self, sign_type, times_to_call=1):
        AbstractRoadObject.__init__(self)
        self.times_to_call_left = times_to_call
        self.SIGN_TYPE = sign_type
    def do(self):
        if self.times_to_call_left > 0:
            self.times_to_call_left -= 1
            return self.get_sign_type_string()
        else:
            return None
    def get_sign_type_string(self):
        return SIGN_TYPE_TO_STR[self.SIGN_TYPE]


class LogicOfTL(Enum):
    ALWAYS_RED = 0
    ALWAYS_GREEN = 1
    START_TIMER_WHEN_REACH = 2
class TL(AbstractRoadObject):
    def __init__(self, tl_topic_name=None, logic=LogicOfTL.START_TIMER_WHEN_REACH,
                 time_before_change_color=5):
        AbstractRoadObject.__init__(self)
        self.time_before_change_color = time_before_change_color
        if logic is LogicOfTL.ALWAYS_RED:
            self.color = TLColor.RED
            self.is_timer_started = True
        elif logic is LogicOfTL.ALWAYS_GREEN:
            self.color = TLColor.GREEN
            self.is_timer_started = True
        else:
            self.color = TLColor.RED
            self.is_timer_started = False

        if tl_topic_name is not None:
            self.gz_pub = rospy.Publisher(tl_topic_name, UInt8, queue_size=10)
            # We can't pub before system started, we should wait few seconds
            TIME_BEFORE_FIRST_PUB = rospy.Duration(5)
            rospy.Timer(TIME_BEFORE_FIRST_PUB, self._gz_pub_cb, oneshot=True)

    def do(self):
        if self.is_timer_started is False:
            self.is_timer_started = True
            self._gz_pub_cb()
            rospy.Timer(rospy.Duration(self.time_before_change_color),
                        self._change_tl_cb, oneshot=True)
        return self.get_tl_type_string()

    def _gz_pub_cb(self, event=None):
        if self.color == TLColor.RED:
            self.gz_pub.publish(1)
        elif self.color == TLColor.GREEN:
            self.gz_pub.publish(2)
        else:
            self.gz_pub.publish(0)

    def _change_tl_cb(self, event):
        self.color = TLColor.GREEN
        self._gz_pub_cb()

    def get_tl_type_string(self):
        return TL_COLOR_TO_STR[self.color]


class Maze(object):
    def __init__(self):
        self.road_objects = dict()
        if SIMULATE_DETECTOR is True:
            self.sign_pub = rospy.Publisher(SIGN_SUB_TOPIC, Detections, queue_size=5)
        else:
            self.sign_pub = None

    def process(self):
        # Process road objects, get their types and publish msg
        msg = Detections()
        crnt_maze_point, orientation = get_cur_maze_pose(pose_listener)

        road_obj_list = self.road_objects.get(crnt_maze_point)
        if road_obj_list is not None:
            for road_obj in road_obj_list:
                msg.detections.append(Detection())
                road_obj.process()
                msg.detections[-1].object_class = road_obj.get_type()
        if SIMULATE_DETECTOR is True:
            self.sign_pub.publish(msg)


class Maze2020V1_1(Maze):
    def __init__(self):
        super(Maze2020V1_1, self).__init__()
        self.road_objects[MazePoint(9, 9)] = [Sign(SignType.BRICK, 1)]
        self.road_objects[MazePoint(9, 8)] = [TL('traffic_light_0_topic')]
        self.road_objects[MazePoint(9, 7)] = [Sign(SignType.ONLY_RIGHT)]
        self.road_objects[MazePoint(7, 7)] = [Sign(SignType.ONLY_RIGHT, 5)]
        self.road_objects[MazePoint(3, 8)] = [Sign(SignType.ONLY_FORWARD, 5)]
        self.road_objects[MazePoint(3, 5)] = [Sign(SignType.BRICK)]
        self.road_objects[MazePoint(5, 4)] = [Sign(SignType.ONLY_FORWARD)]
        self.road_objects[MazePoint(8, 4)] = [Sign(SignType.ONLY_RIGHT)]
        self.road_objects[MazePoint(9, 3)] = [Sign(SignType.FORWARD_OR_LEFT)]
        self.road_objects[MazePoint(9, 1)] = [TL('traffic_light_1_topic')]
        self.road_objects[MazePoint(5, 0)] = [TL('traffic_light_2_topic')]
        self.road_objects[MazePoint(4, 0)] = [TL('traffic_light_3_topic')]
        self.road_objects[MazePoint(5, 2)] = [TL('traffic_light_4_topic')]
        self.road_objects[MazePoint(4, 2)] = [TL('traffic_light_5_topic')]

class Maze2020V2_1(Maze):
    def __init__(self):
        super(Maze2020V2_1, self).__init__()

class Maze2020V2_2(Maze):
    def __init__(self):
        super(Maze2020V2_2, self).__init__()
        self.road_objects[MazePoint(9, 0)] = [TL('traffic_light_0_topic',
                                              logic=LogicOfTL.START_TIMER_WHEN_REACH,
                                              time_before_change_color=10)]
        self.road_objects[MazePoint(6, 0)] = [Sign(SignType.ONLY_RIGHT)]
        self.road_objects[MazePoint(5, 2)] = [Sign(SignType.ONLY_RIGHT)]
        self.road_objects[MazePoint(7, 3)] = [Sign(SignType.ONLY_LEFT)]
        self.road_objects[MazePoint(5, 5)] = [Sign(SignType.BRICK)]
        self.road_objects[MazePoint(5, 4)] = [Sign(SignType.FORWARD_OR_RIGHT)]
        self.road_objects[MazePoint(3, 4)] = [Sign(SignType.FORWARD_OR_LEFT)]


class Maze2020V2_3(Maze):
    def __init__(self):
        super(Maze2020V2_3, self).__init__()
        self.road_objects[MazePoint(9, 0)] = [TL('traffic_light_0_topic',
                                                 logic=LogicOfTL.START_TIMER_WHEN_REACH,
                                                 time_before_change_color=15)]
        self.road_objects[MazePoint(7, 0)] = [Sign(SignType.ONLY_FORWARD)]
        self.road_objects[MazePoint(6, 0)] = [Sign(SignType.ONLY_FORWARD)]
        self.road_objects[MazePoint(4, 0)] = [Sign(SignType.ONLY_FORWARD)]
        self.road_objects[MazePoint(1, 0)] = [TL(None, LogicOfTL.ALWAYS_RED)]
        self.road_objects[MazePoint(1, 0)] = [TL('traffic_light_1_topic',
                                                 logic=LogicOfTL.START_TIMER_WHEN_REACH,
                                                 time_before_change_color=10)]
        self.road_objects[MazePoint(1, 2)] = [Sign(SignType.ONLY_RIGHT)]
        self.road_objects[MazePoint(2, 3)] = [Sign(SignType.BRICK)]
        self.road_objects[MazePoint(3, 4)] = [Sign(SignType.ONLY_LEFT)]

class Maze2020V2_4(Maze):
    def __init__(self):
        super(Maze2020V2_4, self).__init__()
        self.road_objects[MazePoint(9, 0)] = [TL(None, LogicOfTL.ALWAYS_RED)]
        self.road_objects[MazePoint(8, 0)] = [TL('traffic_light_0_topic'), Sign(SignType.ONLY_FORWARD)]
        self.road_objects[MazePoint(6, 0)] = [Sign(SignType.FORWARD_OR_LEFT, times_to_call=5)]
        self.road_objects[MazePoint(4, 0)] = [Sign(SignType.FORWARD_OR_LEFT, times_to_call=5)]
        self.road_objects[MazePoint(2, 0)] = [Sign(SignType.BRICK, times_to_call=5)]
        self.road_objects[MazePoint(1, 0)] = [Sign(SignType.BRICK, times_to_call=1)]

        self.road_objects[MazePoint(2, 3)] = [Sign(SignType.ONLY_FORWARD, times_to_call=5)]
        self.road_objects[MazePoint(4, 3)] = [Sign(SignType.ONLY_FORWARD, times_to_call=5)]
        self.road_objects[MazePoint(6, 3)] = [TL('traffic_light_1_topic')]
        self.road_objects[MazePoint(7, 3)] = [Sign(SignType.ONLY_LEFT, times_to_call=5)]

        self.road_objects[MazePoint(8, 4)] = [TL('traffic_light_2_topic')]

        self.road_objects[MazePoint(5, 5)] = [Sign(SignType.FORWARD_OR_LEFT)]
        self.road_objects[MazePoint(3, 5)] = [Sign(SignType.FORWARD_OR_RIGHT)]

class Maze2020V2_5(Maze):
    def __init__(self):
        super(Maze2020V2_5, self).__init__()
        self.road_objects[MazePoint(9, 0)] = [TL('traffic_light_0_topic', LogicOfTL.ALWAYS_GREEN)]
        self.road_objects[MazePoint(8, 0)] = [TL('traffic_light_0_topic', LogicOfTL.ALWAYS_GREEN)]
        self.road_objects[MazePoint(6, 0)] = [Sign(SignType.BRICK, 5)]
        self.road_objects[MazePoint(5, 0)] = [Sign(SignType.BRICK, 5)]
        self.road_objects[MazePoint(5, 2)] = [Sign(SignType.ONLY_RIGHT, 5)]
        self.road_objects[MazePoint(6, 3)] = [TL('traffic_light_1_topic', LogicOfTL.ALWAYS_GREEN)]
        self.road_objects[MazePoint(7, 3)] = [Sign(SignType.ONLY_LEFT, 5)]
        self.road_objects[MazePoint(8, 4)] = [TL('traffic_light_2_topic', LogicOfTL.ALWAYS_GREEN)]
        self.road_objects[MazePoint(6, 5)] = [Sign(SignType.FORWARD_OR_RIGHT, 5)]
        self.road_objects[MazePoint(5, 5)] = [Sign(SignType.FORWARD_OR_RIGHT, 5)]
        self.road_objects[MazePoint(3, 5)] = [Sign(SignType.ONLY_RIGHT, 5)]
        self.road_objects[MazePoint(2, 6)] = [TL('traffic_light_3_topic', LogicOfTL.ALWAYS_GREEN)]

class Maze2020V2_6(Maze):
    def __init__(self):
        super(Maze2020V2_6, self).__init__()

if __name__ == "__main__":
    rospy.init_node("test_solver", log_level=rospy.INFO)
    SIGN_SUB_TOPIC = rospy.get_param('~detected_sign_type_topic', "detected_sign_type")
    WORLD_NAME = rospy.get_param('~world_name')
    WORLD_MINOR_VERSION = rospy.get_param('~world_minor_version', 0)
    SIMULATE_DETECTOR = rospy.get_param('~simulate_detector', False)

    if WORLD_NAME == 'maze2020v1':
        test_maze = Maze2020V1_1()
        print('Maze2020V1_1 test was activated')
    elif WORLD_NAME == 'maze2020v2' and WORLD_MINOR_VERSION == 1:
        test_maze = Maze2020V2_1()
        print('Maze2020V2_1 test was activated')
    elif WORLD_NAME == 'maze2020v2' and WORLD_MINOR_VERSION == 2:
        test_maze = Maze2020V2_2()
        print('Maze2020V2_2 test was activated')
    elif WORLD_NAME == 'maze2020v2' and WORLD_MINOR_VERSION == 3:
        test_maze = Maze2020V2_3()
        print('Maze2020V2_3 test was activated')
    elif WORLD_NAME == 'maze2020v2' and WORLD_MINOR_VERSION == 4:
        test_maze = Maze2020V2_4()
        print('Maze2020V2_4 test was activated')
    elif WORLD_NAME == 'maze2020v2' and WORLD_MINOR_VERSION == 5:
        test_maze = Maze2020V2_5()
        print('Maze2020V2_5 test was activated')
    else:
        test_maze = Maze()
        print('Test was not activated: ', WORLD_NAME, WORLD_MINOR_VERSION, 'are wrong.')

    sleep(1)
    pose_listener = tf.TransformListener()
    sleep(1)

    while not rospy.is_shutdown():
        test_maze.process()
        rospy.sleep(0.5)
