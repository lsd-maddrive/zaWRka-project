#!/usr/bin/env python3
from lxml import etree
import copy, numpy
from enum import Enum
from json_converter import *

class Point:
    def __init__(self, pos_x, pos_y, pos_z):
        self.x = pos_x
        self.y = pos_y
        self.z = pos_z
    def getString(self):
        return str(self.x) + " " + str(self.y) + " " + str(self.z) + " 0 0 0"

class SignOrientation(Enum):
    RIGHT_TOP = 0
    RIGHT_BOT = 1
    LEFT_TOP = 2
    LEFT_BOT = 3

# Constants
WALL_WIDTH = float(0.01)
WALL_HEGHT = float(0.5)
WALL_SPAWN_Z = float(0.25)

# Files pathes
# For correct use, you should run export GAZEBO_RESOURCE_PATH=path/to/media
SIGN_PATH = "media/stop_sign.sdf"
BOX_PATH = "box.world"
EMPTY_WORLD_PATH = "empty_world.world"

# Signs materials
class SignsImages(Enum):
    STOP = "SignsImages/Stop"
    ONLY_FORWARD = "SignsImages/OnlyForward"
    ONLY_RIGHT = "SignsImages/OnlyRight"
    ONLY_LEFT = "SignsImages/OnlyLeft"
    FORWARD_OR_RIGHT = "SignsImages/ForwardOrRight"
    FORWARD_OR_LEFT = "SignsImages/ForwardOrLeft"


class SdfCreator:
    def __init__(self, start, finish, cellsAmount, cellsSize, mapSize):
        """ 
        @brief Constructor that create empty world with defined config 
        """
        self.__setConfig(start, finish, cellsAmount, cellsSize, mapSize)
        self.__create_empty_world()


    def showTree(self):
        """ 
        @brief Print on console xml tree of current world 
        """
        print(etree.tostring(self.SDF_ROOT, pretty_print=True))


    def writeWorldToFile(self, fileName):
        """ 
        @brief Write current world to file
        """
        f = open(fileName, 'wb')
        f.write(etree.tostring(self.SDF_ROOT, pretty_print=True))


    def addWall(self, point1, point2):
        """ 
        @brief Spawn wall (only vertical or horizontal)
        @param point1 - list of map coordinate (x, y) (from 0 to SIZE_X)
        @param point2 - list of map coordinate (x, y) (from 0 to SIZE_Y)
        @note few notes:
        1. wall must be horizontal or vertical (too lazy to work with 
        rotation angles)
        2. param do not take into account offset
        """
        print("wall with pos:", point1, point2)
        center_x = numpy.mean([point1[0], point2[0]]) 
        center_y = numpy.mean([point1[1], point2[1]])
        center_point = Point(center_x, center_y, WALL_SPAWN_Z)

        # vertical
        if point1[0] == point2[0]:
            wall_length = abs(point1[1] - point2[1])
            wall_size = Point(WALL_WIDTH, wall_length, WALL_HEGHT)
        # horizontal
        elif point1[1] == point2[1]:
            wall_length = abs(point1[0] - point2[0])
            wall_size = Point(wall_length, WALL_WIDTH, WALL_HEGHT)
        else:
            return
        self.__spawnBox(center_point, wall_size)


    def addBox(self, cellIndexes):
        """ 
        @brief Spawn box with cell size in middle of cell
        @param cellIndexes - index of cell (from 0 to CELLS_AMOUNT - 1)
        """
        boxSize = Point(self.CELLS_SIZE[0], self.CELLS_SIZE[1], WALL_HEGHT)
        pose_x = cellIndexes[0] * boxSize.x + boxSize.x / 2
        pose_y = cellIndexes[1] * boxSize.y + boxSize.y / 2
        self.__spawnBox(Point(pose_x, pose_y, WALL_HEGHT), boxSize)


    def addSign(self, position, signType):
        """ 
        @param position - array with x and y position on map
        @param signType - string with value from SignsTypes class
        @note we can figure out orientation from position
        """
        if signType == SignsTypes.STOP.value:
            signImage = SignsImages.STOP
        elif signType == SignsTypes.ONLY_FORWARD.value:
            signImage = SignsImages.ONLY_FORWARD
        elif signType == SignsTypes.ONLY_LEFT.value:
            signImage = SignsImages.ONLY_LEFT
        elif signType == SignsTypes.ONLY_RIGHT.value:
            signImage = SignsImages.ONLY_RIGHT
        elif signType == SignsTypes.FORWARD_OR_LEFT.value:
            signImage = SignsImages.FORWARD_OR_LEFT
        elif signType == SignsTypes.FORWARD_OR_RIGHT.value:
            signImage = SignsImages.FORWARD_OR_RIGHT
        else:
            print("Error: sign type is ", signType)
            return
        self.__spawnSign(position, signImage)


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
        box_position.x = self.START_X - box_position.x
        box_position.y = - self.START_Y + box_position.y
        self.__setBoxParams(box_root, box_position, box_size)
        self.SDF_ROOT.find("world").insert(0, copy.deepcopy(box_root) )


    def __spawnSign(self, position, signImage):
        """ 
        @brief Spawn box in defined position
        @param position - array with x and y position on map (high level 
            abstraction), in other words, start offset is not taken into 
            account.
        @param signImage - object of SignsImages class
        @note You can spawn it in 4 variants (see SignOrientation)
        """
        if (position[0] % self.CELLS_SIZE[0] <= self.CELLS_SIZE[0]/2) and \
           (position[1] % self.CELLS_SIZE[1] <= self.CELLS_SIZE[1]/2):
            orientation = SignOrientation.LEFT_BOT
        elif (position[0] % self.CELLS_SIZE[0] >= self.CELLS_SIZE[0]/2) and \
             (position[1] % self.CELLS_SIZE[1] <= self.CELLS_SIZE[1]/2):
            orientation = SignOrientation.RIGHT_BOT
        elif (position[0] % self.CELLS_SIZE[0] <= self.CELLS_SIZE[0]/2) and \
             (position[1] % self.CELLS_SIZE[1] >= self.CELLS_SIZE[1]/2):
            orientation = SignOrientation.LEFT_TOP
        elif (position[0] % self.CELLS_SIZE[0] >= self.CELLS_SIZE[0]/2) and \
             (position[1] % self.CELLS_SIZE[1] >= self.CELLS_SIZE[1]/2):
            orientation = SignOrientation.RIGHT_TOP
        print("sign stop with pos:", position, orientation, signImage)
        self.sign_counter += 1
        sign_root = etree.parse(SIGN_PATH).getroot()
        position[0] = self.START_X - position[0]
        position[1] = - self.START_Y + position[1]
        self.__setSignParams(sign_root, position, orientation, signImage)
        self.SDF_ROOT.find("world").insert(0, copy.deepcopy(sign_root) )


    def __setSignParams(self, sign_root, position, orientation, signImage):
        """ 
        @brief Set sign desired parameters
        """
        # Calculate sign position
        if orientation is SignOrientation.LEFT_BOT:
            roll = 1.57
            yaw = 1.57
            imagePos = [position[0] - 0.025, position[1] - 0.00]
        elif orientation is SignOrientation.RIGHT_BOT:
            roll = 0
            yaw = 1.57
            imagePos = [position[0] + 0.00, position[1] + 0.025]
        elif orientation is SignOrientation.LEFT_TOP:
            roll = 0
            yaw = -1.57
            imagePos = [position[0] + 0.00, position[1] - 0.025]
        elif orientation is SignOrientation.RIGHT_TOP:
            roll = 1.57
            yaw = -1.57
            imagePos = [position[0] + 0.025, position[1] - 0.00]

        # Set sign position
        signName = "unit_stop_sign_" + str(self.sign_counter)
        columnPos = "{0} {1} 0.35 0.00 0.0 0.0".format(position[0], position[1])
        signPos =   "{0} {1} 0.95 1.57 {2} {3}".format(position[0], position[1], yaw, roll)
        fondPos =   "{0} {1} 0.95 1.57 {2} {3}".format(imagePos[0], imagePos[1], yaw, roll)
        imagePos =  "{0} {1} 0.95 1.57 {2} {3}".format(imagePos[0], imagePos[1], yaw, roll)
        sign_root.set("name", signName)
        sign_root[0][1].text = columnPos
        sign_root[1][1].text = signPos
        sign_root[2][1].text = fondPos
        sign_root[3][1].text = imagePos

        # Set sign image
        sign_root[2].find("visual").find("material").find("script").find("name").text = "Gazebo/White"
        sign_root[3].find("visual").find("material").find("script").find("name").text = signImage.value


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

        self.CELLS_SIZE = list()
        for axe in range(0, 2):
            if( (cellsSize[axe] >= MIN_CELL_SIZE) and (cellsSize[axe] <= MAX_CELL_SIZE)):
                self.CELLS_SIZE.append(cellsSize[axe])
            else:
                self.CELLS_SIZE.append(DEFAULT_CELL_SIZE)

        if ((mapSize[0] >= MIN_MAP_SIZE) and (mapSize[0] <= MAX_MAP_SIZE)):
            self.SIZE_X = mapSize[0]
        else:
            self.SIZE_X = DEFAULT_MAP_SIZE
        if ((mapSize[1] >= MIN_MAP_SIZE) and (mapSize[1] <= MAX_MAP_SIZE)):
            self.SIZE_Y = mapSize[1]
        else:
            self.SIZE_Y = DEFAULT_MAP_SIZE

        if ((start[0] >= 0) and (start[0] <= self.SIZE_X)):
            self.START_X = start[0]
        else:
            self.START_X = DEFAULT_POSE
        if ((start[1] >= 0) and (start[1] <= self.SIZE_X)):
            self.START_Y = start[1]
        else:
            self.START_Y = DEFAULT_POSE

        print("World settings are:") 
        print("- start: [", self.START_X, self.START_Y,  "]")
        print("- finish: [ don't work now]")
        print("- cells amount: [ don't work now]")
        print("- cells size: [", self.CELLS_SIZE[0], self.CELLS_SIZE[1], "]")
        print("- map size: [", self.SIZE_X, self.SIZE_Y, WALL_HEGHT, "]")


    def __create_empty_world(self):
        """ 
        @brief Create sdf tree for empty world from file
        """
        self.SDF_ROOT = etree.parse(EMPTY_WORLD_PATH).getroot()

    # Variables:
    box_counter = 0
    sign_counter = 0

