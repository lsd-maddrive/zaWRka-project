#!/usr/bin/env python
from time import sleep

import rospy
import tf
from std_msgs.msg import UInt8
from mad_detector.msg import Detection, Detections

from maze import MazePoint
from solver import map_to_maze, SIGN_SUB_TOPIC, TLColor, SignType, \
                   TL_COLOR_TO_STR, SIGN_TYPE_TO_STR


def get_cur_maze_pose(pose_listener):
    try:
        trans = pose_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
    except:
        point = None
    return point


class RoadState(object):
    """
    Singleton of test state
    """
    sign_type = SignType.NO_SIGN
    prev_sign_type = SignType.NO_SIGN

    tl_color = TLColor.UNKNOWN
    prev_tl_color = TLColor.UNKNOWN

    @staticmethod
    def set_sign_type(sign_type):
        RoadState.sign_type = sign_type
    @staticmethod
    def get_sign_type():
        return RoadState.sign_type
    @staticmethod
    def get_sign_type_string():
        return SIGN_TYPE_TO_STR[RoadState.sign_type]
    @staticmethod
    def is_sign_changed():
        return RoadState.sign_type == RoadState.prev_sign_type
    @staticmethod
    def update_prev_sign_type():
        RoadState.prev_sign_type = RoadState.sign_type

    @staticmethod
    def set_tl_type(tl_color):
        RoadState.tl_color = tl_color
    @staticmethod
    def get_tl_type():
        return RoadState.tl_color
    @staticmethod
    def get_tl_type_string():
        return TL_COLOR_TO_STR[RoadState.tl_color]
    @staticmethod
    def is_tl_changed():
        return RoadState.tl_color == RoadState.prev_tl_color
    @staticmethod
    def update_prev_tl_type():
        RoadState.prev_tl_color = RoadState.tl_color


class AbstractRoadObject(object):
    def __init__(self, _obj_point):
        self._obj_point = _obj_point
        self._is_it_called_before = False

    def process(self, cur_maze_pose):
        if cur_maze_pose is None:
            return
        if self._obj_point == cur_maze_pose and self._is_it_called_before is False:
            self.do()
            self._is_it_called_before = True

    def do(self):
        pass


class Sign(AbstractRoadObject):
    def __init__(self, obj_point, sign_type):
        AbstractRoadObject.__init__(self, obj_point)
        self.SIGN_TYPE = sign_type
    def do(self):
        RoadState.set_sign_type(self.SIGN_TYPE)


class TL(AbstractRoadObject):
    def __init__(self, obj_point, tl_topic_name):
        AbstractRoadObject.__init__(self, obj_point)
        self.gz_pub = rospy.Publisher(tl_topic_name, UInt8, queue_size=10)
        self.color = TLColor.RED
        rospy.Timer(rospy.Duration(5), self._pub_cb, oneshot=True)

    def do(self, time=7.5):
        RoadState.set_tl_type(TLColor.RED)
        self.color = RoadState.get_tl_type()
        self._pub_cb()
        rospy.Timer(rospy.Duration(time), self._change_tl_cb, oneshot=True)

    def _pub_cb(self, event=None):
        if self.color == TLColor.RED:
            self.gz_pub.publish(1)
        elif self.color == TLColor.GREEN:
            self.gz_pub.publish(2)
        else:
            self.gz_pub.publish(0)

    def _change_tl_cb(self, event):
        RoadState.set_tl_type(TLColor.GREEN)
        self.gz_pub.publish(RoadState.get_tl_type().value)
        self.color = RoadState.get_tl_type()
        self._pub_cb()


rospy.init_node("test_solver", log_level=rospy.INFO)
sign_pub = rospy.Publisher(SIGN_SUB_TOPIC, Detections, queue_size=5)


if __name__ == "__main__":
    sleep(1)
    pose_listener = tf.TransformListener()
    sleep(1)
    road_objects = list((TL(MazePoint(9, 8), 'traffic_light_0_topic'),
                         Sign(MazePoint(9, 7), SignType.ONLY_RIGHT),
                         Sign(MazePoint(7, 7), SignType.ONLY_RIGHT),
                         Sign(MazePoint(3, 8), SignType.FORWARD_OR_RIGHT),
                         Sign(MazePoint(3, 5), SignType.BRICK),
                         Sign(MazePoint(5, 4), SignType.ONLY_FORWARD),
                         Sign(MazePoint(8, 4), SignType.ONLY_RIGHT),
                         Sign(MazePoint(9, 3), SignType.FORWARD_OR_LEFT),
                         TL(MazePoint(9, 1), 'traffic_light_1_topic'),
                         TL(MazePoint(5, 0), 'traffic_light_2_topic'),
                         TL(MazePoint(4, 0), 'traffic_light_3_topic'),
                         TL(MazePoint(5, 2), 'traffic_light_4_topic'),
                         TL(MazePoint(4, 2), 'traffic_light_5_topic')))

    while not rospy.is_shutdown():
        # Process road objects
        cur_maze_point = get_cur_maze_pose(pose_listener)
        RoadState.set_sign_type(SignType.NO_SIGN)
        for road_obj in road_objects:
            road_obj.process(cur_maze_point)

        # Create and publish msg
        msg = Detections()

        msg.detections.append(Detection())
        msg.detections[-1].object_class = RoadState.get_tl_type_string()

        msg.detections.append(Detection())
        msg.detections[-1].object_class = RoadState.get_sign_type_string()

        sign_pub.publish(msg)

        # Show info about changed status
        if not RoadState.is_tl_changed():
            RoadState.update_prev_tl_type()
            rospy.loginfo("Test: TL has been set to %s", RoadState.get_tl_type_string())
        if not RoadState.is_sign_changed():
            RoadState.update_prev_sign_type()
            rospy.loginfo("Test: Sign has been set to %s", RoadState.get_sign_type_string())

        rospy.sleep(0.5)
