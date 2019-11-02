#!/usr/bin/env python3
class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def getStringData(self):
        return "[{0}, {1}]".format(self.x, self.y)

class Vector3D:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def getStringData(self):
        return "[{0}, {1}, {2}]".format(self.x, self.y, self.z)

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
