#!/usr/bin/env python3
from data_structures import *

class Start:
    def __init__(self, point):
        self.start = point

class Finish:
    def __init__(self, point):
        self.finish = point

class Wall:
    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
    def getStringData(self):
        return "[{0}, {1}], [{2}, {3}]".format(self.point1.x, self.point1.y, 
                                               self.point2.x, self.point2.y)

class Box:
    def __init__(self, point):
        self.point = point

class Sign:
    def __init__(self, point, signType, orientation):
        self.point = point
        self.type = signType
        self.orientation = orientation
