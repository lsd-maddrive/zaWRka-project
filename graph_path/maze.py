import numpy as np
import pygame
import itertools as it
from pygame.locals import *
import time

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def get_tuple(self):
        return (self.x, self.y)

    def get_array(self):
        return np.array([self.x, self.y])

    def __str__(self):
        return '[{}; {}]'.format(self.x, self.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

to_directions = [
                    Point(0, 1),
                    Point(1, 0),
                    Point(0, -1),
                    Point(-1, 0)
                ]

from_direction_letters = [
                            'U',
                            'R',
                            'D',
                            'L',
                            '-'
                        ]

from_directions = { 
                from_direction_letters[0] : Point(0, -1),
                from_direction_letters[1] : Point(-1, 0),
                from_direction_letters[2] : Point(0, 1),
                from_direction_letters[3] : Point(1, 0)
            }

def getLetterFromDirPnt(fromDirPnt):
    for direction in from_directions:
        if from_directions[direction] == fromDirPnt:
            return direction

    return None

# With dir letter
class PointDir:
    def __init__(self, x=0, y=0, d='-'):
        self.x = x
        self.y = y
        self.d = d

    def get_tuple(self):
        return (self.x, self.y, self.d)

    def __str__(self):
        return '[{}; {}; {}]'.format(self.x, self.y, self.d)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.d == other.d
    
    def __hash__(self):
        return hash((self.x, self.y, self.d))


def getManhattanDistance(cNode, oNode):
    dist = np.absolute(oNode.coord.get_array() - cNode.coord.get_array())
    return dist[0] + dist[1]


class Node:

    node_idx_cntr = 0
    
    dir_deltas = [Point(0, 1),
                  Point(1, 0),
                  Point(0, -1),
                  Point(-1, 0)]

    NEXT_IDX_LEFT  = 2
    NEXT_IDX_FRWD  = 1
    NEXT_IDX_RGHT  = 0

    def __init__(self, pnt_coord):
        # Up, right, down, left
        self.map_neighbours = [0, 0, 0, 0]
        
        self.coord          = pnt_coord
        self.next_nodes     = [None, None, None, None]

        self.dirNeighbours = [None, None, None]
        self.dirLetr = '-'

        self.idx = -1
        self.signChecked = False

    def __str__(self):
        return 'id: {}/{} ({}/{})'.format(self.idx, self.dirLetr, self.signChecked, self.coord.get_array())

    def getSrcDir(self, cPnt, pPnt):
        for direction in from_directions:
            if from_directions[direction] == (cPnt - pPnt):
                return direction

        return None

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.idx == other.idx and self.dirLetr == other.dirLetr

        return NotImplemented

    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.idx < other.idx

        return NotImplemented
    
    def __hash__(self):
        return hash((self.idx, self.dirLetr))

    def show_info(self):
        print('id: {}/{}:'.format(self.idx, self.dirLetr))
        
        nghbr = self.dirNeighbours[0]
        if nghbr:
            print('  Neighbour R: {}/{}'.format(nghbr.idx, nghbr.dirLetr))
        # else:
            # print('  Neighbour R: None')

        
        nghbr = self.dirNeighbours[1]
        if nghbr:
            print('  Neighbour F: {}/{}'.format(nghbr.idx, nghbr.dirLetr))
        # else:
            # print('  Neighbour F: None')

        
        nghbr = self.dirNeighbours[2]
        if nghbr:
            print('  Neighbour L: {}/{}'.format(nghbr.idx, nghbr.dirLetr))
        # else:
            # print('  Neighbour L: None')


    def setMainNode(self):
        self.idx            = Node.node_idx_cntr
        Node.node_idx_cntr  += 1


class Maze:

    START_NODE_ID   = 1
    END_NODE_ID     = 2

    def __init__(self, structure):
        self.height, self.width = structure.shape
        self.idx_set = set(it.product(range(self.width), range(self.height)))
        self.maze_array = structure

        self.nodes = {}
        self.edges = {}
        self.node_cntr = 0

        self.screen = None

        for x in range(self.height):
            for y in range(self.width):
                point = Point(x, y)
                elem = self.get_maze_element(point)

                if self.is_element_vacant(elem):
                    node = Node(point)

                    for i, delta in enumerate(to_directions):
                        neighbour_pnt = point + delta
                        neighbour = self.get_maze_element(neighbour_pnt)
                        
                        if neighbour is not None and self.is_element_vacant(neighbour):
                            node.map_neighbours[i] = 1

                        # if self.is_element_vacant(neighbour):
                        #     drctn = node.getSrcDir(neighbour_pnt, point)
                        #     if drctn is None:
                        #         print('Fault')

                    if elem == Maze.START_NODE_ID:
                        self.start_node = node
                        self.nodes[point] = node
                        node.setMainNode()
                        continue

                    if elem == Maze.END_NODE_ID:
                        self.end_node   = node
                        self.nodes[point] = node
                        node.setMainNode()
                        continue

                    if sum(node.map_neighbours) == 2 and \
                        ((node.map_neighbours[0] == 1 and node.map_neighbours[2] == 1) or \
                            (node.map_neighbours[1] == 1 and node.map_neighbours[3] == 1)):
                        self.edges[point] = node
                    else:
                        self.nodes[point] = node
                        node.setMainNode()

        self.new_nodes_list = {}

    # Convert to extended nodes
    def getDirNeighbours(self, pnt, fromDirPnt):

        if pnt in self.edges:
            # print('Edge found, skip it from {} to {}'.format(pnt, pnt + fromDirPnt))
            return self.getDirNeighbours(pnt + fromDirPnt, fromDirPnt)

        currNode = self.nodes[pnt]

        if currNode == self.start_node:
            return None

        fromDirLtr = getLetterFromDirPnt(fromDirPnt)
        # print('Node "{}" //{}" found on {}'.format(currNode, fromDirLtr, pnt))
        
        newPntDir = PointDir(pnt.x, pnt.y, fromDirLtr)
        
        if newPntDir in self.new_nodes_list:
            # print('But already found on dictionary')
            return self.new_nodes_list[newPntDir]

        newNode = Node(pnt)
        newNode.idx = currNode.idx
        newNode.dirLetr = fromDirLtr

        # Update target node definition
        if newNode.idx == self.end_node.idx:
            self.end_node = newNode

        # time.sleep(2)

        self.new_nodes_list[newPntDir] = newNode

        rightDir = self.getRightDirPnt(fromDirPnt)
        nextPnt = pnt + rightDir
        if self.isPntValid(nextPnt):
            newNode.dirNeighbours[0] = self.getDirNeighbours(nextPnt, rightDir)
        else:
            newNode.dirNeighbours[0] = None

        forwardDir = self.getForwardDirPnt(fromDirPnt)
        nextPnt = pnt + forwardDir
        if self.isPntValid(nextPnt):
            newNode.dirNeighbours[1] = self.getDirNeighbours(nextPnt, forwardDir)
        else:
            newNode.dirNeighbours[1] = None

        leftDir = self.getLeftDirPnt(fromDirPnt)
        nextPnt = pnt + leftDir
        if self.isPntValid(nextPnt):
            newNode.dirNeighbours[2] = self.getDirNeighbours(nextPnt, leftDir)
        else:
            newNode.dirNeighbours[2] = None

        return newNode


    def getRightDirPnt(self, fromDirPnt):
        toDirPnt = Point(fromDirPnt.y, -fromDirPnt.x)
        return toDirPnt

    def getLeftDirPnt(self, fromDirPnt):
        toDirPnt = Point(-fromDirPnt.y, fromDirPnt.x)
        return toDirPnt

    def getForwardDirPnt(self, fromDirPnt):
        toDirPnt = fromDirPnt
        return toDirPnt

    def calcHeuristic(self, nextNode):
        return getManhattanDistance(nextNode, self.end_node)

    def nextPreprocessing(self):
        self.start_node.dirNeighbours[1] = self.getDirNeighbours(self.start_node.coord + to_directions[0], to_directions[0])

        # for elem in self.new_nodes_list:
            # self.new_nodes_list[elem].show_info()



    def is_element_vacant(self, elem):
        if elem == 0 or elem == 1 or elem == 2:
            return True

        return False

    def isPntValid(self, p):
        if p.x < 0 or p.x >= self.width:
            return False

        if p.y < 0 or p.y >= self.height:
            return False

        elem = self.get_maze_element(p)
        if not self.is_element_vacant(elem):
            return False

        return True

    # Return    
    #   8 - occupied
    #   1 - start
    #   2 - target
    #   0 - free
    def get_maze_element(self, p):
        if p.x < 0 or p.x >= self.width:
            return None

        if p.y < 0 or p.y >= self.height:
            return None

        return self.maze_array[self.height-p.y-1][p.x];

    ### RENDERING ###
    def render_getCellWidth(self):
        return self.screen.get_width() / self.width

    def render_Map2Canvas(self, pnt):
        cellWidth = self.render_getCellWidth()

        pnt[1] = self.height - pnt[1]
        pnt = pnt * cellWidth + np.array([cellWidth/2, -cellWidth/2])
        pnt = pnt.astype(np.int32)

        return pnt

    def render_maze(self):
        cell_colors = (255, 255, 255), (0, 255, 0), (128, 128, 255)
        
        if self.screen is None:
            self.screen = pygame.display.set_mode((420, 420))
        
        self.screen.fill((0, 0, 0))

        def render_get_cell_rect(coordinates, maze):
            cell_margin = 2

            x, y = coordinates
            y = maze.height - 1 - y
            cell_width = maze.render_getCellWidth()
            adjusted_width = cell_width - cell_margin
            return pygame.Rect(x * cell_width + cell_margin / 2,
                               y * cell_width + cell_margin / 2,
                               adjusted_width, adjusted_width)

        font = pygame.font.Font(pygame.font.get_default_font(), 12)

        # White for edges
        for pnt in self.edges:
            coord = pnt.get_array()
            self.screen.fill(cell_colors[0], render_get_cell_rect(coord, self))

        # Green for nodes
        for pnt in self.nodes:
            coord = pnt.get_array()
            rect = render_get_cell_rect(coord, self)

            if self.nodes[pnt] == self.start_node or self.nodes[pnt] == self.end_node:
                self.screen.fill(cell_colors[2], rect)
            else:
                self.screen.fill(cell_colors[1], rect)

            text = font.render("{}".format(self.nodes[pnt].idx), True, (0, 0, 0))
            text_rect = text.get_rect()
            text_rect.center = rect.center

            self.screen.blit(text, text_rect)

        pygame.display.update()

        # Render possible directions

