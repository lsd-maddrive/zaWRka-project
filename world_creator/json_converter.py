#!/usr/bin/env python3

"""
This script allow to create json file from data and sdf file from json.
"""

import json
from json_constants import *
from gazebo_sdf import *
from objects import *

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

def create_json_from_gui(filepath: str, objects: list, map_params: MapParams):
    """ 
    @brief Create json file using frontend data
    """
    serialized = {    
        'objects': [],
        'map_params': map_params.serialize()
    }
    
    for obj_type, objs in objects.items():
        if type(objs) is list:
            for obj in objs:
                serialized['objects'] += [obj.serialized()] 
        else:
            serialized['objects'] += [objs.serialized()] 

    print(serialized)

    try:
        with open(filepath, "w") as fp:
            json.dump(serialized, fp, indent=2)
    except:
        print("Failed to write to file {}".format(filePath))


def create_sdf_from_gui(start, finish, cellsAmount, cellsSize, mapSize,
                         boxes, walls, signs, filePath):
    """ 
    @brief Create sdf file using frontend data
    """
    start = start.convertFromGuiToJson(cellsSize).data
    finish = finish.convertFromGuiToJson(cellsSize).data

    sdfCreator = SdfCreator(start, finish, cellsAmount, cellsSize, mapSize)
    for wall in walls:
        wall = wall.convertFromGuiToJson(cellsSize)
        sdfCreator.addWall(wall)
    for sign in signs:
        sign = sign.convertFromGuiToJson(cellsSize)
        sdfCreator.addSign(sign)
    try:
        sdfCreator.writeWorldToFile(filePath)
    except:
        print("Error: incorrect sdf file name!")

def create_sdf_from_json(jsonFileName, sdfFileName):
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
    try:
        sdfCreator.writeWorldToFile(sdfFileName)
    except:
        print("Error: incorrect sdf file name!")


def load_frontend_from_json(fileName):
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

