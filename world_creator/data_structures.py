#!/usr/bin/env python3
from copy import deepcopy
class Vector2D:
    def __init__(self, arg1=None, arg2=None):
        if arg1 is None:
            self.BuildEmpty()
        elif arg2 is None:
            self.BuildFromArray(arg1)
        else:
            self.BuildFromVeriables(arg1, arg2)
    def __truediv__(self, other):
        answer = deepcopy(self)
        answer.x = self.x / other
        answer.y = self.y / other
        return answer
    def __sub__(self, other):
        answer = deepcopy(self)
        answer.x = self.x - other.x
        answer.y = self.y - other.y
        return answer
    def __str__(self):
        return "[{0}, {1}]".format(self.x, self.y)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def BuildEmpty(self):
        self.x = 0
        self.y = 0
    def BuildFromVeriables(self, x, y):
        self.x = x
        self.y = y
    def BuildFromArray(self, arr):
        self.x = arr[0]
        self.y = arr[1]
    def getListData(self):
        return list([self.x, self.y])

class Vector3D:
    def __init__(self, arg1=None, arg2=None, arg3=None):
        if arg1 is None:
            self.BuildEmpty()
        elif arg2 is None or arg3 is None:
            self.BuildFromArray(arg1)
        else:
            self.BuildFromVeriables(arg1, arg2, arg3)
    def __truediv__(self, other):
        answer = deepcopy(self)
        answer.x = self.x / other
        answer.y = self.y / other
        answer.z = self.z / other
        return answer
    def __sub__(self, other):
        answer = deepcopy(self)
        answer.x = self.x - other.x
        answer.y = self.y - other.y
        answer.z = self.z - other.z
        return answer
    def __str__(self):
        return "[{0}, {1}, {2}]".format(self.x, self.y, self.z)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z
    def BuildEmpty(self):
        self.x = 0
        self.y = 0
        self.z = 0
    def BuildFromVeriables(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def BuildFromArray(self, arr):
        self.x = arr[0]
        self.y = arr[1]
        self.z = arr[2]
    def getListData(self):
        return list([self.x, self.y, self.z])

class Point3D(Vector3D):
    def getString(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.z) + " 0 0 0"

class Point2D(Vector2D):
    pass

class Size2D(Vector2D):
    pass

class Size3D(Vector3D):
    def getString(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.z) + " 0 0 0"
