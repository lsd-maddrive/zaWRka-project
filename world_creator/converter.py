#!/usr/bin/env python3

"""
This script allow to create json file from data and sdf file from json.
"""

import json
import logging as log
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

def serialize_2_json(filepath: str, objects: dict, map_params: MapParams):
    serialized = {    
        'version': APP_VERSION,
        'map_params': map_params.serialize(),
        'objects': []
    }
    
    for obj_type, objs in objects.items():
        if type(objs) is list:
            for obj in objs:
                serialized['objects'] += [obj.serialized()] 
        else:
            serialized['objects'] += [objs.serialized()] 

    log.debug(serialized)

    try:
        with open(filepath, "w") as fp:
            json.dump(serialized, fp, indent=2)
    except:
        log.error("Failed to write to file {}".format(filePath))

def deserialize_from_json(filepath: str, objects: dict):
    map_params = None
    
    with open(filepath, "r") as fp:
        data = json.load(fp)
    
    if data['version'] != APP_VERSION:
        raise Exception('Invalid file version!')
    
    map_params = MapParams.deserialize(data['map_params'])
    
    for obj in data['objects']:
        deser_obj = Object.deserialize(obj)

        if deser_obj:
            if deser_obj.TYPE == ObjectType.START:
                objects[deser_obj.TYPE] = deser_obj
            else:
                objects[deser_obj.TYPE] += [deser_obj]
    
    log.info(map_params)
    
    return map_params

def create_sdf(filepath: str, objects: dict, map_params: MapParams):
    sdfCreator = SdfCreator(map_params)

    for obj_type, objs in objects.items():
        if type(objs) is list:
            for obj in objs:
                sdfCreator.addObject(obj)
        else:
            sdfCreator.addObject(objs)

    try:
        sdfCreator.writeWorldToFile(filepath)
    except:
        log.error("Failed to write WORLD to file")

# def create_sdf_from_json(jsonFileName, sdfFileName):
#     """ 
#     Create sdf world using json data
#     """
#     read_file = open(jsonFileName, "r")
#     data = json.load(read_file)

#     sdfCreator = SdfCreator(Point2D(data.get(JsonNames.START)),
#                             Point2D(data.get(JsonNames.FINISH)),
#                             Point2D(data.get(JsonNames.CELLS_AMOUNT)),
#                             Size2D(data.get(JsonNames.CELLS_SIZE)),
#                             Size2D(data.get(JsonNames.SIZE)))
#     for obj in data.get(JsonNames.OBJECTS):
#         if obj.get(JsonNames.NAME) == JsonNames.BOX:
#             position = Point2D(obj.get(JsonNames.POSITION))
#             sdfCreator.addBox(position)
#         elif obj.get(JsonNames.NAME) == JsonNames.WALL:
#             point1 = obj.get(JsonNames.POINT_1)
#             point2 = obj.get(JsonNames.POINT_2)
#             wall = Wall(Point2D(point1), Point2D(point2))
#             sdfCreator.addWall(wall)
#         elif obj.get(JsonNames.NAME) == JsonNames.SIGN:
#             position = obj.get(JsonNames.POSITION)
#             imgType = obj.get(JsonNames.SIGN_TYPE)
#             sign = Sign(Point2D(position), imgType)
#             sdfCreator.addSign(sign)
#     try:
#         sdfCreator.writeWorldToFile(sdfFileName)
#     except:
#         print("Error: incorrect sdf file name!")



