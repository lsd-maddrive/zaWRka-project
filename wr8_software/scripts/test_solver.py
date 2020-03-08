#!/usr/bin/env python
import math
from enum import Enum
from time import sleep

import rospy
import tf
from std_msgs.msg import UInt8
from mad_detector.msg import Detection, Detections

from maze import MazePoint
from solver import map_to_maze, map_to_gz, TF_SUB_TOPIC, WL_SUB_TOPIC, SIGN_SUB_TOPIC, TFColor, SignType

TF_COLOR_TO_MSG = {
    TFColor.UNKNOWN : " ",
    TFColor.GREEN : "traffic_light_green",
    TFColor.RED : "traffic_light_red",
    }

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

    def __init__(self, tf_topic_name):
        self.pub = rospy.Publisher(tf_topic_name, UInt8, queue_size=10)
        self.color = TFColor.RED
        rospy.Timer(rospy.Duration(5), self._pub_cb, oneshot=True)

    def do(self, time = 7.5):
        TF.tf_color = TFColor.RED
        self.color = TF.tf_color
        self._pub_cb()
        rospy.Timer(rospy.Duration(time), self._change_tf_cb, oneshot=True)

    def _pub_cb(self, event=None):
        if self.color == TFColor.RED:
            self.pub.publish(1)
        elif self.color == TFColor.GREEN:
            self.pub.publish(2)
        else:
            self.pub.publish(0)

    def _change_tf_cb(self, event):
        TF.tf_color = TFColor.GREEN
        self.pub.publish(TF.tf_color.value)
        self.color = TF.tf_color
        self._pub_cb()


class Test(object):
    def __init__(self, point, obj):
        self.point = point
        self.is_it_called_before = False
        #self.is_it_missed_before = False
        self.obj = obj

    def process(self, cur_maze_pose):
        if cur_maze_pose is None:
            return
        if self.point == cur_maze_pose and self.is_it_called_before == False:
            self.obj.do()
            self.is_it_called_before = True


rospy.init_node("test_solver", log_level=rospy.INFO)
tf_pub = rospy.Publisher(TF_SUB_TOPIC, Detections, queue_size=5)
wl_pub = rospy.Publisher(WL_SUB_TOPIC, UInt8, queue_size=5)
sign_pub = rospy.Publisher(SIGN_SUB_TOPIC, UInt8, queue_size=5)

if __name__ == "__main__":
    sleep(1)
    pose_listener = tf.TransformListener()
    sleep(1)
    tests = list()
    tests.append(Test(MazePoint(9, 8), TF('traffic_light_0_topic')))
    tests.append(Test(MazePoint(9, 7), SignStop()))
    tests.append(Test(MazePoint(7, 7), SignRight()))
    tests.append(Test(MazePoint(3, 8), SignForwardOrRight()))
    tests.append(Test(MazePoint(3, 5), SignStop()))
    tests.append(Test(MazePoint(5, 4), SignForward()))
    tests.append(Test(MazePoint(8, 4), SignRight()))
    tests.append(Test(MazePoint(9, 3), SignForwardOrLeft()))
    tests.append(Test(MazePoint(9, 1), TF('traffic_light_1_topic')))
    tests.append(Test(MazePoint(5, 0), TF('traffic_light_2_topic')))
    tests.append(Test(MazePoint(4, 0), TF('traffic_light_3_topic')))
    tests.append(Test(MazePoint(5, 2), TF('traffic_light_4_topic')))
    tests.append(Test(MazePoint(4, 2), TF('traffic_light_5_topic')))

    while not rospy.is_shutdown():
        cur_maze_point = get_cur_maze_pose(pose_listener)
        Sign.sign_type = SignType.NO_SIGN
        
        for test in tests:
            test.process(cur_maze_point)

        if TF.previous_tf_color != TF.tf_color:
            TF.previous_tf_color = TF.tf_color
            msg = Detections()
            msg.detections.append(Detection())
            msg.detections[0].object_class = TF_COLOR_TO_MSG[TF.tf_color]
            tf_pub.publish(msg)
            rospy.loginfo("Test: TF has been set to %s", str(TF.tf_color))
            
        if Sign.previous_sign_type != Sign.sign_type:
            Sign.previous_sign_type = Sign.sign_type
            sign_pub.publish(Sign.sign_type.value)
            rospy.loginfo("Test: Sign has been set to %s", str(Sign.sign_type))

        rospy.sleep(0.5)
