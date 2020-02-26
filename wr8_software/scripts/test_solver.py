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
    trans = pose_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    point = map_to_maze(MazePoint(trans[0][0], trans[0][1]))
    return point

class SignStop(object):
    def do(self):
        sign_pub.publish(SignType.STOP.value)
        print "SignStop activated"

class TF(object):
    def do(self):
        print "TF activated: RED for 15 sec"
        tf_pub.publish(TFColor.RED.value)
        wl_pub.publish(1)
        sleep(15)
        wl_pub.publish(0)
        tf_pub.publish(TFColor.GREEN.value)


class Test(object):
    def __init__(self, point, obj):
        self.point = point
        self.status = False
        self.obj = obj

    def process(self, cur_maze_pose):
        if self.point == cur_maze_pose and self.status == False:
            self.obj.do()
            self.status = True


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
    print "go"
    while not rospy.is_shutdown():
        wl_pub.publish(0)
        tf_pub.publish(TFColor.UNKNOWN.value)
        sign_pub.publish(SignType.NO_SIGN.value)

        cur_maze_point = get_cur_maze_pose(pose_listener)
        print cur_maze_point
        for test in tests:
            test.process(cur_maze_point)
        sleep(3)
