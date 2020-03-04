#!/usr/bin/env python
import math
from enum import Enum
from time import sleep

import rospy
import tf
from std_msgs.msg import UInt8

from maze import MazePoint
from solver import map_to_maze, TF_SUB_TOPIC, WL_SUB_TOPIC, SIGN_SUB_TOPIC, TFColor, SignType


def get_cur_maze_pose(pose_listener):
    try:
        trans = pose_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
    except:
        point = None
    return point

class Sign(object):
    previous_sign_type = SignType.NO_SIGN
    sign_type = SignType.NO_SIGN
    def __init__(self):
        Sign.sign_type = SignType.NO_SIGN

class SignStop(Sign):
    def do(self):
        Sign.sign_type = SignType.STOP

class SignForward(Sign):
    def do(self):
        Sign.sign_type = SignType.ONLY_FORWARD

class SignRight(Sign):
    def do(self):
        Sign.sign_type = SignType.ONLY_RIGHT

class SignLeft(Sign):
    def do(self):
        Sign.sign_type = SignType.ONLY_LEFT

class SignForwardOrRight(Sign):
    def do(self):
        Sign.sign_type = SignType.FORWARD_OR_RIGHT

class SignForwardOrLeft(Sign):
    def do(self):
        Sign.sign_type = SignType.FORWARD_OR_LEFT

class TF(object):
    tf_color = TFColor.UNKNOWN
    wl_status = 0
    previous_tf_color = TFColor.UNKNOWN
    previous_wl_status = 0

    def do(self, time = 7.5):
        TF.tf_color = TFColor.RED
        TF.wl_status = 1
        rospy.Timer(rospy.Duration(time), self._change_tf_and_wl_cb, oneshot=True)

    def _change_tf_and_wl_cb(self, event):
        TF.tf_color = TFColor.GREEN
        TF.wl_status = 0


class Test(object):
    def __init__(self, point, obj):
        self.point = point
        self.is_it_called_before = False
        self.is_it_missed_before = False
        self.obj = obj

    def process(self, cur_maze_pose):
        if cur_maze_pose is None:
            return

        if self.point == cur_maze_pose and self.is_it_missed_before == False:
            self.is_it_missed_before = True
            return

        if self.point == cur_maze_pose and self.is_it_called_before == False:
            self.obj.do()
            self.is_it_called_before = True


rospy.init_node("test_solver", log_level=rospy.DEBUG)
tf_pub = rospy.Publisher(TF_SUB_TOPIC, UInt8, queue_size=5)
wl_pub = rospy.Publisher(WL_SUB_TOPIC, UInt8, queue_size=5)
sign_pub = rospy.Publisher(SIGN_SUB_TOPIC, UInt8, queue_size=5)

if __name__ == "__main__":
    sleep(1)
    pose_listener = tf.TransformListener()
    sleep(1)
    tests = list()
    tests.append(Test(MazePoint(9, 8), TF()))
    tests.append(Test(MazePoint(9, 7), SignStop()))
    tests.append(Test(MazePoint(7, 7), SignRight()))
    tests.append(Test(MazePoint(3, 8), SignForwardOrRight()))
    tests.append(Test(MazePoint(3, 5), SignStop()))
    tests.append(Test(MazePoint(5, 4), SignForward()))
    tests.append(Test(MazePoint(8, 4), SignRight()))
    tests.append(Test(MazePoint(9, 3), SignForwardOrLeft()))
    tests.append(Test(MazePoint(9, 1), TF()))
    tests.append(Test(MazePoint(5, 0), TF()))
    tests.append(Test(MazePoint(4, 0), TF()))
    tests.append(Test(MazePoint(5, 2), TF()))
    tests.append(Test(MazePoint(4, 2), TF()))

    while not rospy.is_shutdown():
        cur_maze_point = get_cur_maze_pose(pose_listener)
        Sign.sign_type = SignType.NO_SIGN
        
        for test in tests:
            test.process(cur_maze_point)

        if TF.previous_wl_status != TF.wl_status:
            TF.previous_wl_status = TF.wl_status
            wl_pub.publish(TF.wl_status)
            rospy.logdebug("White line set to %d", TF.wl_status)

        if TF.previous_tf_color != TF.tf_color:
            TF.previous_tf_color = TF.tf_color
            tf_pub.publish(TF.tf_color.value)
            rospy.logdebug("TF set to %s", str(TF.tf_color))
            
        if Sign.previous_sign_type != Sign.sign_type:
            Sign.previous_sign_type = Sign.sign_type
            sign_pub.publish(Sign.sign_type.value)
            rospy.logdebug("Sign set to %s", str(Sign.sign_type))

        rospy.sleep(1)
