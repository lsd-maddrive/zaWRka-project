#!/usr/bin/env python3
import logging as log
from lxml import etree
import copy, numpy
import math as m
from enum import Enum
import converter
from data_structures import *
from objects import *

# Constants
WALL_WIDTH = float(0.01)
WALL_HEIGHT = float(0.5)
WALL_SPAWN_Z = WALL_HEIGHT / 2

# Files pathes
BOX_PATH = "models/box.sdf"
EMPTY_WORLD_PATH = "models/empty_world.world"

# Signs materials
class SignsModels(Enum):
    STOP = "model://brick-sign"
    ONLY_FORWARD = "model://forward-sign"
    ONLY_RIGHT = "model://right-sign"
    ONLY_LEFT = "model://left-sign"
    FORWARD_OR_RIGHT = "model://forward-right-sign"
    FORWARD_OR_LEFT = "model://forward-left-sign"


class SdfCreator:
    def __init__(self, map_params: MapParams):
        """ 
        @brief Constructor that create empty world with defined config
        """
        # self.__setConfig(start, finish, cellsAmount, cellsSize, mapSize)
        self.__create_empty_world()
        
        self.map_params = map_params

    def showTree(self):
        """ 
        @brief Print on console xml tree of current world 
        """
        log.debug(etree.tostring(self.SDF_ROOT, pretty_print=True))


    def writeWorldToFile(self, fileName):
        """ 
        @brief Write current world to file
        """
        with open(fileName, 'wb') as f:
            f.write(etree.tostring(self.SDF_ROOT, pretty_print=True))

    def addObject(self, obj: Object):
        FUNCTIONS_MAPPING = {
            # ObjectType.WALL: self.addWall,
            ObjectType.SIGN: self.addSign
        }

        if obj.TYPE not in FUNCTIONS_MAPPING:
            log.error('Object type {} is not supported in WORLD generation'.format(obj.TYPE.name))
            return

        FUNCTIONS_MAPPING[obj.TYPE](obj)

    def addWall(self, wall):
        """ 
        @brief Spawn wall (only vertical or horizontal)
        @param wall - object from json
        @note wall must be horizontal or vertical (too lazy to work with
        rotation angles)
        """
        log.debug("wall with pos:", wall)
        center_x = numpy.mean([wall.point1.x, wall.point2.x]) 
        center_y = numpy.mean([wall.point1.y, wall.point2.y])
        center_point = Point3D(center_x, center_y, WALL_SPAWN_Z)

        # vertical
        if wall.point1.x == wall.point2.x:
            wall_length = abs(wall.point1.y - wall.point2.y)
            wall_size = Point3D(WALL_WIDTH, wall_length, WALL_HEIGHT)
        # horizontal
        elif wall.point1.y == wall.point2.y:
            wall_length = abs(wall.point1.x - wall.point2.x)
            wall_size = Point3D(wall_length, WALL_WIDTH, WALL_HEIGHT)
        else:
            return
        self.__spawnBox(center_point, wall_size)


    def addBox(self, box):
        """ 
        @brief Spawn box with cell size in middle of cell
        @param box - object from json
        """
        boxSize = Point3D(self.CELLS_SIZE.x, self.CELLS_SIZE.y, WALL_HEIGHT)
        pose_x = box.point.x * boxSize.x + boxSize.x / 2
        pose_y = box.point.y * boxSize.y + boxSize.y / 2
        self.__spawnBox(Point3D(pose_x, pose_y, WALL_HEIGHT), boxSize)

    def __spawnBox(self, box_position, box_size):
        """ 
        @brief Spawn box with defined size in defined position
        @note You can spawn it in 2 variants:
        1. box: on center of cell in template [odd; odd], 
            for example [3; 1], [3; 3], [3; 5]
        2. wall: on center of cell in template [even; odd] or [odd; even], 
            for example [1; 0], [2; 1], [4; 3]
        @param box_position - position in high level abstraction, in other 
            words, start offset is not taken into account.
        """
        self.box_counter += 1
        box_root = etree.parse(BOX_PATH).getroot()
        box_position.x = self.START.x - box_position.x
        box_position.y = - self.START.y + box_position.y
        self.__setBoxParams(box_root, box_position, box_size)
        self.SDF_ROOT.find("world").insert(0, copy.deepcopy(box_root) )


    ORIENTATIONS_2_YAW_ANGLE = {
        CellQuarter.LEFT_BOT: 0,
        CellQuarter.RIGHT_BOT: m.pi/2,
        CellQuarter.LEFT_TOP: m.pi*3/2,
        CellQuarter.RIGHT_TOP: m.pi,
    }
    
    def addSign(self, sign):
        """ 
        @param sign - object from json
        @note we can figure out orientation from position
        """
        SIGN_MODEL_MAP = {
            SignsTypes.STOP.value:              SignsModels.STOP,
            SignsTypes.ONLY_FORWARD.value:      SignsModels.ONLY_FORWARD,
            SignsTypes.ONLY_LEFT.value:         SignsModels.ONLY_LEFT,
            SignsTypes.ONLY_RIGHT.value:        SignsModels.ONLY_RIGHT,
            SignsTypes.FORWARD_OR_LEFT.value:   SignsModels.FORWARD_OR_LEFT,
            SignsTypes.FORWARD_OR_RIGHT.value:  SignsModels.FORWARD_OR_RIGHT,
        }
  
        if sign.type not in SIGN_MODEL_MAP:
            log.error("Error: sign type {} is not supported".format(sign.type))
            return
        
        self.__spawnSign(sign.point, sign.orient, SIGN_MODEL_MAP[sign.type])


    def __spawnSign(self, position, orient, signImage):
        """ 
        @brief Spawn box in defined position
        @param position - Point2D - position on map (high level abstraction),
            in other words, start offset is not taken into account.
        @param signImage - object of SignsModels class
        @note You can spawn it in 4 variants (see SignOrientation)
        """

        ### LEFT/RIGHT_BOT/TOP - in terms of rendered map
        log.debug("sign with pos: {} / {} / {}".format(position, orient, signImage))

        # position.x = self.START.x - position.x
        # position.y = -self.START.y + position.y
      
        # Apply small shift
        if orient == CellQuarter.RIGHT_TOP:
            position.x += 0.75
            position.y += 0.25
        elif orient == CellQuarter.RIGHT_BOT:
            position.x += 0.75
            position.y += 0.75
        elif orient == CellQuarter.LEFT_BOT:
            position.x += 0.25
            position.y += 0.75
        elif orient == CellQuarter.LEFT_TOP:
            position.x += 0.25
            position.y += 0.25
        
        # Swap axes
        position.y = self.map_params.n_cells.y - position.y
        
        # Turn to physical
        position.x = position.x * self.map_params.cell_sz.x
        position.y = position.y * self.map_params.cell_sz.y
        
        yaw_angle = self.ORIENTATIONS_2_YAW_ANGLE[orient]

        sign_root = etree.Element("include")
        uri_elem = etree.Element("uri")
        uri_elem.text = signImage.value
        name_elem = etree.Element("name")
        name_elem.text = "sign_{}".format(self.sign_counter)
        pose_elem = etree.Element("pose")
        pose_elem.text = "{0} {1} 0 0 0 {2}".format(position.x, position.y, yaw_angle)
        sign_root.append( uri_elem )
        sign_root.append( name_elem )
        sign_root.append( pose_elem )

        self.SDF_ROOT.find("world").insert(0, sign_root )
        self.sign_counter += 1


    def __setBoxParams(self, box_root, box_position, box_size):
        """ 
        @brief Set box desired parameters
        @note To avoid collision problem when objects spawn on borders of each
            other, the collision length will reduce on WALL width size
        """
        box_name = "unit_box_" + str(self.box_counter)
        box_position_text = box_position.getString()
        box_visual_size_text = box_size.getString()

        collision_size = box_size
        if collision_size.x > WALL_WIDTH:
            collision_size.x = collision_size.x - WALL_WIDTH
        if collision_size.y > WALL_WIDTH:
            collision_size.y -= WALL_WIDTH
        box_collision_size_text = collision_size.getString()

        box_root.set("name", box_name)
        box_root.find("pose").text = box_position_text
        link = box_root.find("link")
        link.find("collision").find("geometry").find("box").find("size").text = box_collision_size_text
        link.find("visual").find("geometry").find("box").find("size").text = box_visual_size_text

    @staticmethod
    def __controlRange(value, minimum, maximum, default):
        if value >= minimum and value <= maximum:
            return value
        else:
            return default


    def __setConfig(self, start, finish, cellsAmount, cellsSize, mapSize):
        """ 
        @brief Set config from inputs
        """
        MIN_MAP_SIZE = 4
        MAX_MAP_SIZE = 40
        MIN_CELL_SIZE = 0.1
        MAX_CELL_SIZE = 10

        DEFAULT_MAP_SIZE = 18
        DEFAULT_POSE = 17
        DEFAULT_CELL_SIZE = 2

        # Control cells size range
        x = SdfCreator.__controlRange(cellsSize.x, MIN_CELL_SIZE, MAX_CELL_SIZE,
                                      DEFAULT_CELL_SIZE)
        y = SdfCreator.__controlRange(cellsSize.y, MIN_CELL_SIZE, MAX_CELL_SIZE,
                                      DEFAULT_CELL_SIZE)
        self.CELLS_SIZE = Size2D(x, y)

        # Control map size range
        x = SdfCreator.__controlRange(mapSize.x, MIN_MAP_SIZE, MAX_MAP_SIZE,
                                      DEFAULT_MAP_SIZE)
        y = SdfCreator.__controlRange(mapSize.y, MIN_MAP_SIZE, MAX_MAP_SIZE, 
                                      DEFAULT_MAP_SIZE)
        self.MAP_SIZE = Size2D(x, y)

        # Control start range
        x = SdfCreator.__controlRange(start.x, 0,self.MAP_SIZE.x,DEFAULT_POSE)
        y = SdfCreator.__controlRange(start.y, 0,self.MAP_SIZE.y,DEFAULT_POSE)
        self.START = Size2D(x, y)

        log.debug("World settings are:") 
        log.debug("- start:", self.START)
        log.debug("- finish: [ don't support now]")
        log.debug("- cells amount: [ don't support now]")
        log.debug("- cells size:", self.CELLS_SIZE)
        log.debug("- map size:", self.MAP_SIZE)


    def __create_empty_world(self):
        """ 
        @brief Create sdf tree for empty world from file
        """
        self.SDF_ROOT = etree.parse(EMPTY_WORLD_PATH).getroot()

    # Variables:
    box_counter = 0
    sign_counter = 0

