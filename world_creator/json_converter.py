#!/usr/bin/env python3

"""
This script allow to create json file from data and sdf file from json.
"""

import json
from json_constants import *
from gazebo_sdf import *
from objects import *

# File input-output default settings
JSON_DEFAULT_NAME = "data_file.json"
SDF_DEFAULT_NAME = "../wr8_description/worlds/world.world"

# Cheet sheet
"""
Data types:
- JSON (user) level: it uses only map position in meters
- Frontend level: indexes of cells and nodes (they are more conveniente to use
  because QPaint window is divided into cells and real cell sizes in meters
  are not interesting in this abstraction level)
- Low level: gazebo world position - (robot should spawn in (0,0) gazebo
  position, but user think in his abstraction level that start position can be
  any, so backend methods use start position offset)

Frontend v.2 data:
1.  start       indexes (cell)              Start(Point2D)
2.  finish      indexes (cell)              Finish(Point2D)
3.  size        meters                      Size2D
4.  boxes       indexes (cell)              Point2D
5.  walls       indexes (node)              Wall(Point2D, Point2D)
6.  signs       [indexes(half cell)), path] Sign([x, y], path)

Json data:
1.  start       meters                      list([x, y])
2.  finish      meters                      list([x, y])
3.  size        meters                      list([x, y])
4.  boxes       meters                      list([x, y])
5.  walls       meters                      list([x, y], [x, y])
6.  signs       [meters, type]              list([x, y], type)
"""

def create_json_from_gui(start, finish, cellsAmount, cellsSize, mapSize,
                         boxes, walls, signs):
    """ 
    @brief Create json file using frontend data
    """
    objects = list()
    start = start.convertFromGuiToJson(cellsSize)
    finish = finish.convertFromGuiToJson(cellsSize)
    for wall in walls:
        wall = wall.convertFromGuiToJson(cellsSize)
        wall = dict([ (JsonNames.NAME, JsonNames.WALL), 
                   (JsonNames.POINT_1, wall.point1.getListData()),
                   (JsonNames.POINT_2, wall.point2.getListData()) ])
        objects.append(wall)
    for sign in signs:
        sign = sign.convertFromGuiToJson(cellsSize)
        sign = dict([ (JsonNames.NAME, JsonNames.SIGN), 
                      (JsonNames.POSITION, sign.point.getListData()),
                      (JsonNames.SIGN_TYPE, sign.type) ])
        objects.append(sign)
    data = dict([(JsonNames.START, start.getListData()),
                 (JsonNames.FINISH, finish.getListData()),
                 (JsonNames.CELLS_AMOUNT, cellsAmount.getListData()),
                 (JsonNames.CELLS_SIZE, cellsSize.getListData()),
                 (JsonNames.SIZE, mapSize.getListData()),
                 (JsonNames.OBJECTS, objects)])
    write_file = open(JSON_DEFAULT_NAME, "w")
    json.dump(data, write_file, indent=2)


def create_sdf_from_json(jsonFileName=JSON_DEFAULT_NAME, sdfFileName=SDF_DEFAULT_NAME):
    """ 
    Create sdf world using json data
    """
    read_file = open(jsonFileName, "r")
    data = json.load(read_file)

    sdfCreator = SdfCreator(Point2D(data.get(JsonNames.START)),
                            Point2D(data.get(JsonNames.FINISH)),
                            Point2D(data.get(JsonNames.CELLS_AMOUNT)),
                            Size2D(data.get(JsonNames.CELLS_SIZE)),
                            Size2D(data.get(JsonNames.SIZE)))
    for obj in data.get(JsonNames.OBJECTS):
        if obj.get(JsonNames.NAME) == JsonNames.BOX:
            position = Point2D(obj.get(JsonNames.POSITION))
            sdfCreator.addBox(position)
        elif obj.get(JsonNames.NAME) == JsonNames.WALL:
            point1 = obj.get(JsonNames.POINT_1)
            point2 = obj.get(JsonNames.POINT_2)
            wall = Wall(Point2D(point1), Point2D(point2))
            sdfCreator.addWall(wall)
        elif obj.get(JsonNames.NAME) == JsonNames.SIGN:
            position = obj.get(JsonNames.POSITION)
            imgType = obj.get(JsonNames.SIGN_TYPE)
            sign = Sign(Point2D(position), imgType)
            sdfCreator.addSign(sign)
    sdfCreator.writeWorldToFile(sdfFileName)


def load_frontend_from_json(fileName = JSON_DEFAULT_NAME):
    """ 
    Load fronted data from json file
    """
    read_file = open(fileName, "r")
    data = json.load(read_file)

    cellsSize = Size2D(data.get(JsonNames.CELLS_SIZE))

    start = Start(Point2D(data.get(JsonNames.START)))
    start.convertFromJsonToGui(cellsSize)

    finish = Finish(Point2D(data.get(JsonNames.FINISH)))
    finish.convertFromJsonToGui(cellsSize)

    boxes = list()
    walls = Walls()
    signs = Signs()
    for obj in data.get(JsonNames.OBJECTS):
        if obj.get(JsonNames.NAME) == JsonNames.BOX:
            position = obj.get(JsonNames.POSITION)
            boxes.append(position)
        elif obj.get(JsonNames.NAME) == JsonNames.WALL:
            p1 = Point2D(obj.get(JsonNames.POINT_1))
            p2 = Point2D(obj.get(JsonNames.POINT_2))
            wall = Wall(p1, p2)
            wall.convertFromJsonToGui(cellsSize)
            walls.add(wall)
        elif obj.get(JsonNames.NAME) == JsonNames.SIGN:
            position = Point2D(obj.get(JsonNames.POSITION))
            signPath = obj.get(JsonNames.SIGN_TYPE)
            sign = Sign(position, signPath)
            sign.convertFromJsonToGui(cellsSize)
            signs.add(sign)
    return list([start,
                 finish,
                 data.get(JsonNames.SIZE),
                 boxes,
                 walls,
                 signs])

