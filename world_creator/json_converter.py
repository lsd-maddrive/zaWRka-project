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
1.  start       indexes (cell)              list of x and y
2.  end         indexes (cell)              list of x and y
3.  size        meters                      list of x and y
4.  boxes       indexes (cell)              list of x and y
5.  walls       indexes (node)              list of few 2x2 arrays
6.  signs       [inexes(half cell)), path]  list([x, y], path)

Json data:
1.  start       meters                      list of x and y
2.  end         meters                      list of x and y
3.  size        meters                      list of x and y
4.  boxes       meters                      list of x and y
5.  walls       meters                      list of few 2x2 arrays
6.  signs       [meters, type]              list([x, y], type)
"""

# Global variables:
CellsSize = Size2D(float(2), float(2))  #default

# Sign transformation:
def __map_pose_to_half_cell_indexes(mapPose):
    return list([ (mapPose[0] - CellsSize.x/4) / CellsSize.x * 2, 
                  (mapPose[1] - CellsSize.y/4) / CellsSize.y * 2 ])
def __half_cell_indexes_to_map_pose(cellIndexes):
    return list([ CellsSize.x/4 + cellIndexes.x*CellsSize.x/2, 
                  CellsSize.y/4 + cellIndexes.y*CellsSize.y/2 ])

def create_json_from_gui(start, finish, cellsAmount, cellsSize, mapSize,
                         boxes, walls, signs):
    """ 
    @brief Create json file using frontend data
    @param cellsAmount - Size2D
    @param cellsSize - Size2D
    @param mapSize - Size2D
    """
    global CellsSize
    CellsSize = cellsSize
    objects = list()
    start = start.convertFromGuiToJson(CellsSize)
    finish = finish.convertFromGuiToJson(CellsSize)
    for wall in walls:
        wall = wall.convertFromGuiToJson(CellsSize)
        wall = dict([ (JsonNames.NAME, JsonNames.WALL), 
                   (JsonNames.POINT_1, wall.point1.getListData()),
                   (JsonNames.POINT_2, wall.point2.getListData()) ])
        objects.append(wall)
    for sign in signs:
        sign = dict([ (JsonNames.NAME, JsonNames.SIGN), 
                      (JsonNames.POSITION, __half_cell_indexes_to_map_pose(sign[0])),
                      (JsonNames.SIGN_TYPE, sign_path_to_sign_type(sign[1])) ])
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

    sdfCreator = SdfCreator(Point2D(*data.get(JsonNames.START)),
                            Point2D(*data.get(JsonNames.FINISH)),
                            Point2D(*data.get(JsonNames.CELLS_AMOUNT)),
                            Size2D(*data.get(JsonNames.CELLS_SIZE)),
                            Size2D(*data.get(JsonNames.SIZE)))
    for obj in data.get(JsonNames.OBJECTS):
        if obj.get(JsonNames.NAME) == JsonNames.BOX:
            position = Point2D(*obj.get(JsonNames.POSITION))
            sdfCreator.addBox(position)
        elif obj.get(JsonNames.NAME) == JsonNames.WALL:
            point1 = obj.get(JsonNames.POINT_1)
            point2 = obj.get(JsonNames.POINT_2)
            wall = Wall(Point2D(*point1), Point2D(*point2))
            sdfCreator.addWall(wall)
        elif obj.get(JsonNames.NAME) == JsonNames.SIGN:
            position = obj.get(JsonNames.POSITION)
            imgType = obj.get(JsonNames.SIGN_TYPE)
            sign = Sign(Point2D(*position), imgType, "")
            sdfCreator.addSign(sign)
    sdfCreator.writeWorldToFile(sdfFileName)


def load_frontend_from_json(fileName = JSON_DEFAULT_NAME):
    """ 
    Load fronted data from json file
    """
    read_file = open(fileName, "r")
    data = json.load(read_file)
    global CellsSize
    CellsSize = Size2D(data.get(JsonNames.CELLS_SIZE))

    start = Start(Point2D(data.get(JsonNames.START)))
    start.convertFromJsonToGui(CellsSize)

    finish = Finish(Point2D(data.get(JsonNames.FINISH)))
    finish.convertFromJsonToGui(CellsSize)

    boxes = list()
    walls = Walls()
    signs = list()
    for obj in data.get(JsonNames.OBJECTS):
        if obj.get(JsonNames.NAME) == JsonNames.BOX:
            position = obj.get(JsonNames.POSITION)
            boxes.append(position)
        elif obj.get(JsonNames.NAME) == JsonNames.WALL:
            p1 = Point2D(obj.get(JsonNames.POINT_1))
            p2 = Point2D(obj.get(JsonNames.POINT_2))
            wall = Wall(p1, p2)
            wall.convertFromJsonToGui(CellsSize)
            walls.add(wall)
        elif obj.get(JsonNames.NAME) == JsonNames.SIGN:
            position = __map_pose_to_half_cell_indexes(obj.get(JsonNames.POSITION))
            position = Point2D(position)
            position.x = int(position.x)
            position.y = int(position.y)
            signPath = sign_type_to_sign_path(obj.get(JsonNames.SIGN_TYPE))
            signs.append([position, signPath])
    return list([start,
                 finish,
                 data.get(JsonNames.SIZE),
                 boxes,
                 walls,
                 signs])


