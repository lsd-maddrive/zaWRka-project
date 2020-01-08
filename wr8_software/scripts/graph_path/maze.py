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

    def invert_direction(self):
        return Point(-self.x, -self.y)

    # local TF -> ROS: (x1, y1) = (2*y, -2*x)
    # ROS -> local TF: (x, y) = (-y1/2, x1/2)
    def get_ros_point(self):
        ros_x =  2 * self.y
        ros_y = -2 * self.x
        return np.array([ros_x, ros_y])

    def __str__(self):
        return '[{}; {}]'.format(self.x, self.y)

    def __repr__(self):
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

direction_angle = { 
                from_direction_letters[0] : 180,
                from_direction_letters[1] : 90,
                from_direction_letters[2] : 0,
                from_direction_letters[3] : -90
        }
            
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

    # local TF -> ROS: (x1, y1) = (2*y, -2*x)
    # ROS -> local TF: (x, y) = (-y1/2, x1/2)
    def get_ros_point(self):
        ros_x =  2 * self.y
        ros_y = -2 * self.x

        if self.d == from_direction_letters[-1]:
            print('Invalid direction in get_ros_point()!')

        ros_angle_deg = direction_angle[self.d]

        return np.array([ros_x, ros_y, ros_angle_deg])

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
        self.orig_dirNeighb = None
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
        print('id: {}/{}/{}:'.format(self.idx, self.dirLetr, self.coord))
        
        nghbr = self.dirNeighbours[0]
        if nghbr:
            print('  Neighbour R: {}/{}/{}'.format(nghbr.idx, nghbr.dirLetr, nghbr.coord))
        # else:
            # print('  Neighbour R: None')

        
        nghbr = self.dirNeighbours[1]
        if nghbr:
            print('  Neighbour F: {}/{}/{}'.format(nghbr.idx, nghbr.dirLetr, nghbr.coord))
        # else:
            # print('  Neighbour F: None')

        
        nghbr = self.dirNeighbours[2]
        if nghbr:
            print('  Neighbour L: {}/{}/{}'.format(nghbr.idx, nghbr.dirLetr, nghbr.coord))
        # else:
            # print('  Neighbour L: None')

    def is_main(self):
        return self.idx >= 0

    def has_only_way(self):
        return sum(self.map_neighbours) == 1

    def setMainNode(self):
        self.idx = Node.node_idx_cntr
        Node.node_idx_cntr  += 1

    def update_state(self):
        neig_sum = sum(self.map_neighbours)

        if neig_sum == 2 and \
            ((self.map_neighbours[0] + self.map_neighbours[2] == 2) or \
             (self.map_neighbours[1] + self.map_neighbours[3] == 2)):
            return

        self.setMainNode()


class Maze:

    def __init__(self, structure):
        self.height, self.width = structure.shape
        self.idx_set = set(it.product(range(self.width), range(self.height)))
        self.maze_array = structure

        self.nodes = {}
        self.edges = {}
        self.node_cntr = 0

        self.screen = None

        self.new_nodes_list = {}

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

                    node.update_state()
                    # print('Value for point: {} / {} / {}'.format(point, node.map_neighbours, node.is_main()))
                    if node.is_main():
                        self.nodes[point] = node
                    else:
                        self.edges[point] = node

        node_points = list(self.nodes.keys())
        first_node = self.nodes[node_points[0]]

        nonzero_index = first_node.map_neighbours.index(filter(lambda x: x!=0, first_node.map_neighbours)[0])

        first_pnt, to_dir = first_node.coord, to_directions[nonzero_index]
        # print('Start with: {} -> {}'.format(first_pnt, to_dir))
        self._update_neighbours(first_pnt + to_dir, to_dir)       


        for elem in self.new_nodes_list:
            self.new_nodes_list[elem].show_info()


    # Convert to extended nodes
    def _update_neighbours(self, pnt, fromDirPnt):

        if pnt in self.edges:
            print('Edge found, skip it from {} to {}'.format(pnt, pnt + fromDirPnt))
            return self._update_neighbours(pnt + fromDirPnt, fromDirPnt)

        currNode = self.nodes[pnt]

        fromDirLtr = getLetterFromDirPnt(fromDirPnt)
        print('Node "{} / {}" found on {}'.format(currNode, fromDirLtr, pnt))
        
        newPntDir = PointDir(pnt.x, pnt.y, fromDirLtr)
        
        if newPntDir in self.new_nodes_list:
            # print('But already found on dictionary')
            return self.new_nodes_list[newPntDir]

        newNode = Node(pnt)
        newNode.idx = currNode.idx
        newNode.dirLetr = fromDirLtr

        self.new_nodes_list[newPntDir] = newNode

        # Right, Forward, Left
        direction_pnts = self.get_direction_points(fromDirPnt)
        for i, dir_pnt in enumerate(direction_pnts):
            nextPnt = pnt + dir_pnt
            if self.isPntValid(nextPnt):
                newNode.dirNeighbours[i] = self._update_neighbours(nextPnt, dir_pnt)
            else:
                newNode.dirNeighbours[i] = None

        if all(v is None for v in newNode.dirNeighbours):
            # No return as we start new stream back
            self._update_neighbours(pnt, fromDirPnt.invert_direction())

        return newNode

    def get_direction_points(self, fromDirPnt):
        return [Point(fromDirPnt.y, -fromDirPnt.x), fromDirPnt, Point(-fromDirPnt.y, fromDirPnt.x)]

    def calcHeuristic(self, nextNode):
        return getManhattanDistance(nextNode, self.end_node)

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

            #if self.nodes[pnt] == self.start_node or self.nodes[pnt] == self.end_node:
            #    self.screen.fill(cell_colors[2], rect)
            #else:
            self.screen.fill(cell_colors[1], rect)

            text = font.render("{}".format(self.nodes[pnt].idx), True, (0, 0, 0))
            text_rect = text.get_rect()
            text_rect.center = rect.center

            self.screen.blit(text, text_rect)

        pygame.display.update()

        # Render possible directions


if __name__ == "__main__":
    pygame.init()

    structure = [[0, 0, 0, 0, 0, 0, 0],
                 [0, 8, 0, 8, 0, 8, 0],
                 [0, 8, 0, 8, 0, 0, 0],
                 [0, 0, 0, 0, 0, 8, 8],
                 [0, 8, 0, 8, 0, 8, 8],
                 [0, 8, 0, 8, 0, 0, 0],
                 [0, 8, 0, 0, 0, 8, 8]]
    structure = np.array(structure, np.uint8)
    maze = Maze(structure)

    maze.render_maze()

    time.sleep(3)

    print('Done')
