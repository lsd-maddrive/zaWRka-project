#!/usr/bin/env python3
import logging as log
from lxml import etree
import copy
import math as m
from enum import Enum
import converter
from data_structures import *
from objects import *

# Constants
OBJECT_HEIGHT = float(0.5)
OBJECT_SPAWN_Z = OBJECT_HEIGHT / 2

WALL_WIDTH = float(0.01)

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
    # Variables:
    box_counter = 0
    sign_counter = 0
    
    def __init__(self, map_params: MapParams):
        """ 
        @brief Constructor that create empty world with defined config
        """
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
            ObjectType.WALL: self.addWall,
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
        log.debug("wall with pos: {}".format(wall))
        
        wall_center = wall.get_center()
        wall_length = wall.get_phys_length(self.map_params.cell_sz)
        wall_angle = wall.get_angle()
        
        wall_center.y = self.map_params.n_cells.y - wall_center.y
                
        wall_center.x *= self.map_params.cell_sz.x
        wall_center.y *= self.map_params.cell_sz.y
        
        pos_str = '{} {} {} 0 0 {}'.format(wall_center.x, wall_center.y, OBJECT_SPAWN_Z, -wall_angle)
        size_str = '{} {} {}'.format(wall_length, WALL_WIDTH, OBJECT_HEIGHT)

        self.__spawnBox(pos_str, size_str)

    def addBox(self, box):
        """ 
        @brief Spawn box with cell size in middle of cell
        @param box - object from json
        """

        box_center = box.pos
        
        box_center.x += 0.5
        box_center.y += 0.5
        
        box_center.y = self.map_params.n_cells.y - box_center.y
    
        box_center.x *= self.map_params.cell_sz.x
        box_center.y *= self.map_params.cell_sz.y
        
        pos_str = '{} {} {} 0 0 0'.format(box_center.x, box_center.y, OBJECT_SPAWN_Z)
        size_str = '{} {} {}'.format(self.map_params.cell_sz.x, self.map_params.cell_sz.y, OBJECT_HEIGHT)
        
        self.__spawnBox(pos_str, size_str)

    def __spawnBox(self, pos_str, size_str):
        self.box_counter += 1
        box_root = etree.parse(BOX_PATH).getroot()        
        
        box_root.set("name", "box_{}".format(self.box_counter))
        box_root.find("pose").text = pos_str
        link = box_root.find("link")
        link.find("collision").find("geometry").find("box").find("size").text = size_str
        link.find("visual").find("geometry").find("box").find("size").text = size_str
        
        self.SDF_ROOT.find("world").insert(0, box_root)


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
        position.x *= self.map_params.cell_sz.x
        position.y *= self.map_params.cell_sz.y
        
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

    def __create_empty_world(self):
        """ 
        @brief Create sdf tree for empty world from file
        """
        self.SDF_ROOT = etree.parse(EMPTY_WORLD_PATH).getroot()

