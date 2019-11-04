#!/usr/bin/env python3
from data_structures import *
import copy

class Object:
    def getData(self):
        return self.data
    def getListData(self):
        return self.data.getListData()
    def convertFromJsonToGui(self, cellsSize):
        return None
    def convertFromGuiToJson(self, cellsSize):
        return None

class Start(Object):
    def __init__(self, point=None):
        if point is not None:
            self.data = point
        else:
            self.data = Point2D()
    def convertFromJsonToGui(self, cellsSize):
        self.__map_pose_to_cell_indexes(cellsSize)
    def convertFromGuiToJson(self, cellsSize):
        data = copy.deepcopy(self)
        data.__cell_indexes_to_map_pose(cellsSize)
        return data
    def __str__(self):
        return str(self.data)
    def __map_pose_to_cell_indexes(self, cellsSize):
        self.data = Point2D(int((self.data.x - 1) / cellsSize.x), 
                             int((self.data.y - 1) / cellsSize.y) )
    def __cell_indexes_to_map_pose(self, cellsSize):
        self.data = Point2D(self.data.x * cellsSize.x + 1, 
                             self.data.y * cellsSize.y + 1 )

class Finish(Object):
    def __init__(self, point=None):
        if point is not None:
            self.data = point
        else:
            self.data = Point2D()
    def convertFromJsonToGui(self, cellsSize):
        self.__map_pose_to_cell_indexes(cellsSize)
    def convertFromGuiToJson(self, cellsSize):
        data = copy.deepcopy(self)
        data.__cell_indexes_to_map_pose(cellsSize)
        return data
    def __str__(self):
        return str(self.data)
    def getData(self):
        return self.data
    def __map_pose_to_cell_indexes(self, cellsSize):
        self.data = Point2D(int((self.data.x - 1) / cellsSize.x), 
                             int((self.data.y - 1) / cellsSize.y) )
    def __cell_indexes_to_map_pose(self, cellsSize):
        self.data = Point2D(self.data.x * cellsSize.x + 1, 
                             self.data.y * cellsSize.y + 1 )

class Wall():
    def __init__(self, point1=None, point2=None):
        if point1 is not None or point2 is not None:
            self.point1 = point1
            self.point2 = point2
        else:
            self.point1 = Point2D()
            self.point2 = Point2D()
    def getStringData(self):
        return "[{0}, {1}], [{2}, {3}]".format(self.point1.x, self.point1.y, 
                                               self.point2.x, self.point2.y)
    def __str__(self):
        return "[{0}, {1}], [{2}, {3}]".format(self.point1.x, self.point1.y, 
                                               self.point2.x, self.point2.y)
    def getListData(self):
        print("from Wall")
        return list([self.data.point1.getArrayData(),
                     self.data.point2.getArrayData() ])
    def convertFromJsonToGui(self, cellsSize):
        self.__map_pose_to_node_indexes(cellsSize)
    def convertFromGuiToJson(self, cellsSize):
        data = copy.deepcopy(self)
        data.__node_indexes_to_map_pose(cellsSize)
        return data
    def __map_pose_to_node_indexes(self, cellsSize):
        self.point1 = Point2D(int(self.point1.x / cellsSize.x), 
                              int(self.point1.y / cellsSize.y))
        self.point2 = Point2D(int(self.point2.x / cellsSize.x), 
                              int(self.point2.y / cellsSize.y))
    def __node_indexes_to_map_pose(self, cellsSize):
        self.point1 = Point2D(self.point1.x * cellsSize.x, 
                              self.point1.y * cellsSize.y)
        self.point2 = Point2D(self.point2.x * cellsSize.x, 
                              self.point2.y * cellsSize.y)

class Walls(Object):
    def __init__(self, walls=None):
        if walls is None:
            self.data = list()
        else:
            self.data = list([walls])
    def add(self, wall):
        self.data.append(wall)
    def remove(self, wall):
        self.data.remove(wall)
    def __str__(self):
        text = str()
        for wall in self.data:
            text += "\n" + str(wall)
        return text
    def __getitem__(self, i):
        return self.data[i]
    def __len__(self):
        return len(self.data)

class Box(Object):
    def __init__(self, point):
        self.point = point

class Sign(Object):
    def __init__(self, point, signType, orientation):
        self.point = point
        self.type = signType
        self.orientation = orientation
