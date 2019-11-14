#!/usr/bin/env python3
from data_structures import *
import copy
from json_constants import *

from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QIcon, QImage
from PyQt5.QtCore import Qt, QSize

class CellQuarter(Enum):
    RIGHT_TOP = 0
    RIGHT_BOT = 1
    LEFT_TOP = 2
    LEFT_BOT = 3

class MyPainter(QPainter):
    def __init__(self, base=None, cell_sz=Size2D()):
        super().__init__(base)
        
        self.cell_sz = cell_sz

    def fillCell(self, cell_pos, color=QColor(255, 0, 0)):
        self.setBrush(QBrush(color))
        self.drawRect(cell_pos.x * self.cell_sz.x, cell_pos.y * self.cell_sz.y, 
                      self.cell_sz.x, self.cell_sz.y)
    
    def drawWallLine(self, node_pos1, node_pos2, color=QColor(0, 0, 0)):
        self.setPen(QPen(color, 3))
        self.drawLine(node_pos1.x * self.cell_sz.x, node_pos1.y * self.cell_sz.y,
                      node_pos2.x * self.cell_sz.x, node_pos2.y * self.cell_sz.y)
        
    def drawQuarterImg(self, cell, quarter, img_path):
        self.half_cell_sz = self.cell_sz / 2
        
        render_x = cell.x
        render_y = cell.y
        
        if quarter == CellQuarter.RIGHT_TOP or quarter == CellQuarter.RIGHT_BOT:
            render_x += 0.5
        
        if quarter == CellQuarter.RIGHT_BOT or quarter == CellQuarter.LEFT_BOT:
            render_y += 0.5
        
        img = QImage(img_path).scaled(QSize(self.half_cell_sz.x, self.half_cell_sz.y))
        self.drawImage(render_x * self.cell_sz.x, render_y * self.cell_sz.y, img)

        
class Object:
    def getData(self):
        return self.data
    def getListData(self):
        return self.data.getListData()
    def convertFromJsonToGui(self, cellsSize):
        pass
    def convertFromGuiToJson(self, cellsSize):
        return None
    def __str__(self):
        return str(self.data)
    def render(self):
        pass


class Start(Object):
    def __init__(self, point=None):
        self.data = point if point is not None else Point2D()
    def convertFromJsonToGui(self, cellsSize):
        self.__map_pose_to_cell_indexes(cellsSize)
    def convertFromGuiToJson(self, cellsSize):
        data = copy.deepcopy(self)
        data.__cell_indexes_to_map_pose(cellsSize)
        return data
    def __map_pose_to_cell_indexes(self, cellsSize):
        self.data = Point2D(int((self.data.x - cellsSize.x) / cellsSize.x), 
                             int((self.data.y - cellsSize.y) / cellsSize.y) )
    def __cell_indexes_to_map_pose(self, cellsSize):
        self.data = Point2D(self.data.x * cellsSize.x + cellsSize.x, 
                             self.data.y * cellsSize.y + cellsSize.y )
    def render(self, qp):
        qp.fillCell(self.data, color=QColor(255, 0, 0))
    

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
    def __map_pose_to_cell_indexes(self, cellsSize):
        self.data = Point2D(int((self.data.x - cellsSize.x) / cellsSize.x), 
                             int((self.data.y - cellsSize.y) / cellsSize.y) )
    def __cell_indexes_to_map_pose(self, cellsSize):
        self.data = Point2D(self.data.x * cellsSize.x + cellsSize.x, 
                             self.data.y * cellsSize.y + cellsSize.y )


class Wall():
    def __init__(self, point1=None, point2=None):
        if point1 is not None or point2 is not None:
            self.point1 = point1
            self.point2 = point2
        else:
            self.point1 = Point2D()
            self.point2 = Point2D()
    def __str__(self):
        return "[{0}, {1}], [{2}, {3}]".format(self.point1.x, self.point1.y, 
                                               self.point2.x, self.point2.y)
    def getListData(self):
        return list([self.point1.getListData(), self.point2.getListData() ])
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
    def render(self, qp):
        qp.drawWallLine(self.point1, self.point2, color=QColor(0, 0, 0))
        

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


class Sign():
    def __init__(self, point=None, orient=None, signType=None):
        if point is not None or signType is not None:
            self.point = point
            self.type = signType
            self.orient = orient
        else:
            self.point = Point2D
            self.type = str()
    def __str__(self):
        return "[pose = {0}, type = {1}]".format(self.point, self.type)
    def getListData(self):
        return list([ self.point.getListData(), self.signType.getListData() ])
    def convertFromJsonToGui(self, cellsSize):
        self.__map_pose_to_half_cell_indexes(cellsSize)
    def convertFromGuiToJson(self, cellsSize):
        data = copy.deepcopy(self)
        data.__half_cell_indexes_to_map_pose(cellsSize)
        return data
    def __map_pose_to_half_cell_indexes(self, cellsSize):
        self.point = Point2D( (self.point.x - cellsSize.x/4) / cellsSize.x * 2, 
                              (self.point.y - cellsSize.y/4) / cellsSize.y * 2 )
        self.type = sign_type_to_sign_path(self.type)
    def __half_cell_indexes_to_map_pose(self, cellsSize):
        self.point = Point2D( cellsSize.x/4 + self.point.x * cellsSize.x/2, 
                              cellsSize.y/4 + self.point.y * cellsSize.y/2 )
        self.type = sign_path_to_sign_type(self.type)
    def render(self, qp):
        qp.drawQuarterImg(self.point, self.orient, self.type)
        

class Signs(Object):
    def __init__(self, signs=None):
        if signs is None:
            self.data = list()
        else:
            self.data = signs
    def add(self, sign):
        self.data.append(sign)
    def remove(self, sign):
        self.data.remove(sign)
    def __str__(self):
        text = str()
        for wall in self.data:
            text += "\n" + str(wall)
        return text
    def __getitem__(self, i):
        return self.data[i]
    def __len__(self):
        return len(self.data)

