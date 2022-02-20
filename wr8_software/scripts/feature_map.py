#!/usr/bin/env python2
from enum import Enum
from copy import copy, deepcopy

import numpy as np

import rospy
from mad_detector.msg import Detections

from maze import MazePoint

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

TL_TYPE_FROM_STR = {"traffic_light_green": TLColor.GREEN,
                    "traffic_light_red": TLColor.RED}

SIGN_TYPE_FROM_STR = {"brick": SignType.BRICK,
                      "forward": SignType.ONLY_FORWARD,
                      "right": SignType.ONLY_RIGHT,
                      "left": SignType.ONLY_LEFT,
                      "forward_left": SignType.FORWARD_OR_LEFT,
                      "forward_right": SignType.FORWARD_OR_RIGHT}

TL_COLOR_TO_STR = {TLColor.UNKNOWN : " ",
                   TLColor.GREEN : "traffic_light_green",
                   TLColor.RED : "traffic_light_red"}

SIGN_TYPE_TO_STR = {SignType.NO_SIGN : " ",
                    SignType.BRICK : "brick",
                    SignType.ONLY_FORWARD : "forward",
                    SignType.ONLY_RIGHT : "right",
                    SignType.ONLY_LEFT : "left",
                    SignType.FORWARD_OR_LEFT : "forward_left",
                    SignType.FORWARD_OR_RIGHT : "forward_right"}

class NodeStatus(Enum):
    EMPTY = 0
    FILLED = 8

class Orientation(Enum):
    RIGHT = 0
    BOT = 1.57
    LEFT = 3.14
    TOP = 4.71

class Feature(object):
    def __init__(self, type, orientation):
        self.type = 0
        self.orientation = 0
        self.weight = 0

class FeatureFilter:
    def __init__(self, size=5):
        self.signs = []
        self.tls = []
        self.count = 0
        self.size = size
        for idx in range(size):
            self.signs.append(SignType.NO_SIGN)
            self.tls.append(TLColor.UNKNOWN)

    def update(self, sign, tl):
        self.signs[self.count] = copy(sign)
        self.tls[self.count] = copy(tl)
        self.count = self.count + 1 if (self.count < self.size - 1) else 0

    def get_sign(self):
        counts = list()
        for idx in range(len(SignType)):
            counts.append(self.signs.count(SignType(idx)))
        idx = counts.index(max(counts))
        return SignType(idx)

    def get_tl_color(self):
        counts = list()
        for idx in range(len(TLColor)):
            counts.append(self.tls.count(TLColor(idx)))
        idx = counts.index(max(counts))
        return TLColor(idx)

class FeatureHandler(object):
    """ Feature handler
    - msgs with features from detectors,
    - info about map and current position and orientation
    And produce:
    - info of actual features existing"""
    def __init__(self, structure=np.array(0)):
        """
        Set initial info about map using np.array.
        Now map size is constant, but in future it can be expanded in real time.
        """
        self.structure = structure
        self.sign_type = SignType.NO_SIGN
        self.tl_color = TLColor.UNKNOWN
        self.filter = FeatureFilter(rospy.get_param('~feature_handler/median_filter_size'))

        sign_min_size = rospy.get_param('~sign_min_size')
        tl_min_size = rospy.get_param('~tl_min_size')
        self.SIGN_MIN_SIZE_X = sign_min_size[0]
        self.SIGN_MIN_SIZE_Y = sign_min_size[1]
        self.TL_MIN_SIZE_X = tl_min_size[0]
        self.TL_MIN_SIZE_Y = tl_min_size[1]

    def update(self, detected_features, crnt_node, robot_orientation):
        """
        1. We always think that TL may be only in the current node
        because there is a white line to check it.
        2. We always think that Sign may be only in the next node
        because we always delete Maze local target when reach it.
        3. We ignore signs out of the map

        detected_features - list of mad_detector.msg/Detection
        """
        # Check parameters
        if detected_features is None:
            rospy.logerr("feature_map: update(): wrong detected_features parameter")
            return
        elif crnt_node is None:
            rospy.logerr("feature_map: update(): wrong detected_features parameter")
            return
        elif robot_orientation is None:
            rospy.logerr("feature_map: update(): wrong robot_orientation parameter")
            return

        robot_orientation = FeatureHandler._determine_orientation(robot_orientation)
        next_node = self._calculate_next_node([crnt_node.x, crnt_node.y], robot_orientation)
        self.tl_color = TLColor.UNKNOWN
        self.sign_type = SignType.NO_SIGN

        for detection in detected_features:
            sign_type = SIGN_TYPE_FROM_STR.get(detection.object_class)
            tl_type = TL_TYPE_FROM_STR.get(detection.object_class)
            if sign_type is not None and \
               next_node is not None and \
               self._check_sign_size(detection.size_px):
                self.sign_type = sign_type
            elif tl_type is not None and \
                 self._check_tl_size(detection.size_px):
                self.tl_color = tl_type

            # just log
            if sign_type is not None and next_node is None:
                rospy.logwarn("feature_map: I heard a SIGN with type %s" +
                              "on %s but it can't exist because next" +
                              "node is None.", str(sign_type), next_node)
            elif sign_type is not None and self._check_sign_size(detection.size_px) is False:
                rospy.logwarn('%s has been ignored. Size [%i, %i] should be at least [%i, %i].',
                              sign_type,
                              detection.size_px.x, detection.size_px.y,
                              self.SIGN_MIN_SIZE_X, self.SIGN_MIN_SIZE_Y)
            elif tl_type is not None and self._check_tl_size(detection.size_px) is False:
                rospy.logwarn('%s has been ignored. Size [%i, %i] should be at least [%i, %i].',
                              tl_type,
                              detection.size_px.x, detection.size_px.y,
                              self.TL_MIN_SIZE_X, self.TL_MIN_SIZE_Y)
            elif self.sign_type is not SignType.NO_SIGN:
                rospy.loginfo('%s has been applied. Next node is [%f, %f] and it is ok.',
                              sign_type,
                              next_node[0], next_node[1])

        self.filter.update(self.sign_type, self.tl_color)

    def get_sign_status(self, crnt_robot_node, robot_orientation):
        return self.filter.get_sign()

    def get_tl_status(self, crnt_robot_node, robot_orientation):
        return self.filter.get_tl_color()

    def _check_sign_size(self, size):
        if size.x > self.SIGN_MIN_SIZE_X and size.y > self.SIGN_MIN_SIZE_Y:
            return True
        else:
            return False

    def _check_tl_size(self, size):
        if size.x > self.TL_MIN_SIZE_X and size.y > self.TL_MIN_SIZE_Y:
            return True
        else:
            return False

    @staticmethod
    def _determine_orientation(robot_orientation):
        if robot_orientation < 0:
            robot_orientation += 6.28
        if robot_orientation < np.pi/4 or robot_orientation > 7*np.pi/4:
            robot_orientation = Orientation.RIGHT
        elif robot_orientation < 3*np.pi/4:
            robot_orientation = Orientation.TOP
        elif robot_orientation < 5*np.pi/4:
            robot_orientation = Orientation.LEFT
        else:
            robot_orientation = Orientation.BOT
        return robot_orientation

    def _calculate_next_node(self, crnt_node_indexes, robot_orientation):
        next_node = copy(crnt_node_indexes)
        row_idx = self.structure.shape[0] - 1 - int(next_node[1])
        col_idx = int(next_node[0])
        if robot_orientation == Orientation.RIGHT:
            next_node[0] += 1
        elif robot_orientation == Orientation.BOT:
            next_node[1] -= 1
        elif robot_orientation == Orientation.LEFT:
            next_node[0] -= 1
        elif robot_orientation == Orientation.TOP:
            next_node[1] += 1
        if int(next_node[0]) >= self.structure.shape[1] or int(next_node[0]) < 0:
            return None
        elif int(next_node[1]) >= self.structure.shape[0] or int(next_node[1]) < 0:
            return None
        elif self.structure[row_idx, col_idx] == NodeStatus.FILLED.value:
            return None
        return next_node

