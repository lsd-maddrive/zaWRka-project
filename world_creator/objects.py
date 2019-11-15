#!/usr/bin/env python3
from data_structures import *
import copy
from json_constants import *
import logging as log
import converter

from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QIcon, QImage
from PyQt5.QtCore import Qt, QSize

class ObjectType(Enum):
    START = 10,
    WALL = 11,
    BOX = 12,
    SQUARE = 13,
    SIGN = 14,
    TRAFFIC_LIGHT = 15

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
        self.setPen(Qt.NoPen)
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


class MapParams:
    def __init__(self, n_cells: Size2D, cell_sz: Size2D):
        self.n_cells = n_cells
        self.cell_sz = cell_sz
        self.phys_size = Size2D(self.n_cells.x * self.cell_sz.x, 
                                self.n_cells.y * self.cell_sz.y)
        print("World cells: count={0},size={1}".format(self.n_cells, self.cell_sz))

    def serialize(self):
        data = {
            'cell_cnt': self.n_cells.as_list(),
            'cell_sz': self.cell_sz.as_list()
        }
        
        return data
    
    def __str__(self):
        return 'Map params: count({}) / size({})'.format(self.n_cells, self.cell_sz)
    
    @staticmethod
    def deserialize(data: dict):
        return MapParams(Point2D.from_list(data['cell_cnt']), Point2D.from_list(data['cell_sz']))

        
class Object:
    def render(self):
        pass
    def serialized(self):
        pass
    @staticmethod
    def deserialize(data: dict):
        if data['name'] not in SERIALIZATION_SUPPORT:
            log.error('Object type \'{}\' not found'.format(data['name']))
            return None
        
        return SERIALIZATION_SUPPORT[data['name']].deserialize(data)
    

class Start(Object):
    TYPE = ObjectType.START
    
    def __init__(self, pos: Point2D):
        self.pos = pos
        
    def __str__(self):
        return "[({}) pos = {}]".format(type(self), self.pos)

    def render(self, qp):
        qp.fillCell(self.pos, color=QColor(255, 0, 0))
        
    def serialized(self):
        for name, _class in SERIALIZATION_SUPPORT.items():
            if type(self) == _class:
                break
        
        data = {
            'name': name,
            'pos': self.pos.as_list()
        }

        return data
    
    @staticmethod
    def deserialize(data: dict):
        return Start(Point2D.from_list(data['pos']))
    

class Wall():
    TYPE = ObjectType.WALL

    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2
    def __str__(self):
        return "[({}) p1 = {}, p2 = {}]".format(type(self), self.p1, self.p2)
    
    def render(self, qp):
        qp.drawWallLine(self.p1, self.p2, color=QColor(0, 0, 0))
        
    def serialized(self):
        for name, _class in SERIALIZATION_SUPPORT.items():
            if type(self) == _class:
                break
        
        data = {
            'name': name,
            'pnts': self.p1.as_list() + self.p2.as_list()
        }
        
        return data

    @staticmethod
    def deserialize(data: dict):
        return Wall(Point2D.from_list(data['pnts'][0:2]), 
                    Point2D.from_list(data['pnts'][2:4]))
    
    
class Sign(Object):
    TYPE = ObjectType.SIGN
    
    def __init__(self, point, orient, signType):
        self.point = point
        self.type = signType
        self.orient = orient

    def __str__(self):
        return "[({}) pose = {}, orient = {}, type = {}]".format(type(self), self.point, self.orient, self.type)
    
    def render(self, qp):
        qp.drawQuarterImg(self.point, self.orient, self.type)
    
    def serialized(self):
        for name, _class in SERIALIZATION_SUPPORT.items():
            if type(self) == _class:
                break
        
        data = {
            'name': name,
            'pos': self.point.as_list(),
            'orient': self.orient.value,
            'type': self.type
        }
        
        return data        
    
    @staticmethod
    def deserialize(data: dict):
        return Sign(Point2D.from_list(data['pos']), 
                    CellQuarter(data['orient']),
                    data['type'])
    
    

class Box(Object):
    # TODO - planned
    def __init__(self, point):
        self.point = point
        
        
SERIALIZATION_SUPPORT = {
    'start': Start,
    'wall': Wall,
    'sign': Sign 
}
    