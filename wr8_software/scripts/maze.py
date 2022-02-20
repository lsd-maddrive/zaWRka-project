#!/usr/bin/env python2
import numpy as np
try:
    import pygame
    from pygame.locals import *
    pygame_imported = True
except:
    print('Failed to import pygame - render disabled')
    pygame_imported = False
import itertools as it
import time
from enum import Enum

from Queue import PriorityQueue

import logging


class MazePoint:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def get_tuple(self):
        return (self.x, self.y)

    def as_array(self):
        return np.array([self.x, self.y])

    def invert_direction(self):
        return MazePoint(-self.x, -self.y)

    def __str__(self):
        return '[{}; {}]'.format(self.x, self.y)

    def __repr__(self):
        return '[{}; {}]'.format(self.x, self.y)

    def __add__(self, other):
        return MazePoint(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return MazePoint(self.x - other.x, self.y - other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


to_directions = [
    MazePoint(0, 1),
    MazePoint(1, 0),
    MazePoint(0, -1),
    MazePoint(-1, 0)
]


from_direction_letters = [
    'U',
    'R',
    'D',
    'L',
    '-'
]

g_direction_angle = {
    from_direction_letters[0]: 180,
    from_direction_letters[1]: 90,
    from_direction_letters[2]: 0,
    from_direction_letters[3]: 270
}

from_directions = {
    from_direction_letters[0]: MazePoint(0, -1),
    from_direction_letters[1]: MazePoint(-1, 0),
    from_direction_letters[2]: MazePoint(0, 1),
    from_direction_letters[3]: MazePoint(1, 0)
}



def getLetterFromDirPnt(fromDirPnt):
    for direction in from_directions:
        if from_directions[direction] == fromDirPnt:
            return direction

    return None

# With dir letter


class PointDir(object):
    def __init__(self, x=0, y=0, d='-'):
        self.x = x
        self.y = y
        self.d = d

    def get_tuple(self):
        return (self.x, self.y, self.d)

    def as_point(self):
        return MazePoint(self.x, self.y)

    def as_array(self):
        return np.array([self.x, self.y])

    def __str__(self):
        return '[{}; {}; {}]'.format(self.x, self.y, self.d)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.d == other.d

    def __hash__(self):
        return hash((self.x, self.y, self.d))


class InitialDirection(Enum):
    DOWN = 0
    LEFT = 1
    UP = 2
    RIGHT = 3


class MazePointDir(PointDir):
    def __init__(self, x, y, dir):
        assert isinstance(dir, InitialDirection)
        
        dir_letter = from_direction_letters[dir.value]
        super(MazePointDir, self).__init__(x, y, dir_letter)


def get_manhattan_dist(cur_node, oNode):
    dist = np.absolute(oNode.coord.as_array() - cur_node.coord.as_array())
    return dist[0] + dist[1]


class Node:

    node_idx_cntr = 0

    dir_deltas = [MazePoint(0, 1),
                  MazePoint(1, 0),
                  MazePoint(0, -1),
                  MazePoint(-1, 0)]

    def __init__(self, pnt_coord):
        # Up, right, down, left
        self.map_neighbours = [0, 0, 0, 0]

        self.coord = pnt_coord
        self.next_nodes = [None, None, None, None]

        self.dirNeighbours = [None, None, None]
        self.dir_limitations = [0, 0, 0]

        self.orig_dirNeighb = None
        self.dirLetr = '-'

        self.idx = -1
        self.signChecked = False

    def __str__(self):
        return 'id: {}/{} ({}/{})'.format(self.idx, self.dirLetr, self.signChecked, self.coord.as_array())

    def is_turn(self):
        return sum(x is not None for x in self.dirNeighbours) < 2

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
            print('  Neighbour R: {}/{}/{}'.format(nghbr.idx,
                                                   nghbr.dirLetr, nghbr.coord))
        # else:
            # print('  Neighbour R: None')

        nghbr = self.dirNeighbours[1]
        if nghbr:
            print('  Neighbour F: {}/{}/{}'.format(nghbr.idx,
                                                   nghbr.dirLetr, nghbr.coord))
        # else:
            # print('  Neighbour F: None')

        nghbr = self.dirNeighbours[2]
        if nghbr:
            print('  Neighbour L: {}/{}/{}'.format(nghbr.idx,
                                                   nghbr.dirLetr, nghbr.coord))
        # else:
            # print('  Neighbour L: None')

    def is_main(self):
        return self.idx >= 0

    def has_only_way_node(self):
        return sum(self.map_neighbours) == 1

    def _set_main_node(self):
        self.idx = Node.node_idx_cntr
        Node.node_idx_cntr += 1

    def update_state(self):
        neig_sum = sum(self.map_neighbours)

        if neig_sum == 2 and \
            ((self.map_neighbours[0] + self.map_neighbours[2] == 2) or
             (self.map_neighbours[1] + self.map_neighbours[3] == 2)):
            return

        self._set_main_node()


class Maze:

    def __init__(self, structure, edge_walls=[]):
        self.logger = logging.getLogger(self.__class__.__name__)

        self.maze_array = np.array(structure, np.uint8)
        self.maze_walls = edge_walls

        self.height, self.width = self.maze_array.shape
        self.idx_set = set(it.product(range(self.width), range(self.height)))

        self.nodes = {}
        self.directed_nodes = {}

        self.edges = {}
        self.node_cntr = 0

        self._target_node = None
        self._local_target_node = None
        self._current_node = None

        for x in range(self.width):
            for y in range(self.height):
                point = MazePoint(x, y)
                elem = self.get_maze_element(point)

                if self.is_element_vacant(elem):
                    node = Node(point)

                    for i, delta in enumerate(to_directions):
                        neighbour_pnt = point + delta
                        neighbour = self.get_maze_element(neighbour_pnt)

                        if neighbour is None:
                            continue

                        if not self.is_element_vacant(neighbour):
                            continue

                        if self._is_movement_prohibited(point, neighbour_pnt):
                            continue

                        node.map_neighbours[i] = 1

                    node.update_state()
                    #print('Value for point: {} / {} / {}'.format(point,
                    #                                             node.map_neighbours, node.is_main()))
                    if node.is_main():
                        self.nodes[point] = node
                    else:
                        self.edges[point] = node

        node_points = list(self.nodes.keys())
        first_node = self.nodes[node_points[0]]

        nonzero_index = np.nonzero(first_node.map_neighbours)[0][0]
        self.logger.debug(nonzero_index)

        # nonzero_index = first_node.map_neighbours.index(filter(lambda x: x!=0, first_node.map_neighbours)[0])

        first_pnt, to_dir = first_node.coord, to_directions[nonzero_index]
        # print('Start with: {} -> {}'.format(first_pnt, to_dir))
        self._update_neighbours(first_pnt + to_dir, to_dir)

        # for elem in self.directed_nodes:
        #     self.directed_nodes[elem].show_info()

    def _is_movement_prohibited(self, pnt, neighb_pnt):
        for wall_p1, wall_p2 in self.maze_walls:
            self.logger.debug('Check {}/{} vs {}/{}'.format(pnt, neighb_pnt, wall_p1, wall_p2))

            if pnt == wall_p1 and neighb_pnt == wall_p2:
                return True

            if pnt == wall_p2 and neighb_pnt == wall_p1:
                return True

        return False

    # Convert to extended nodes
    def _update_neighbours(self, pnt, fromDirPnt):

        if pnt in self.edges:
            # print('Edge found, skip it from {} to {}'.format(pnt, pnt + fromDirPnt))
            return self._update_neighbours(pnt + fromDirPnt, fromDirPnt)

        currNode = self.nodes[pnt]

        fromDirLtr = getLetterFromDirPnt(fromDirPnt)
        # print('Node "{} / {}" found on {}'.format(currNode, fromDirLtr, pnt))

        newPntDir = PointDir(pnt.x, pnt.y, fromDirLtr)

        if newPntDir in self.directed_nodes:
            # print('But already found on dictionary')
            return self.directed_nodes[newPntDir]

        newNode = Node(pnt)
        newNode.idx = currNode.idx
        newNode.dirLetr = fromDirLtr

        self.directed_nodes[newPntDir] = newNode

        # Right, Forward, Left
        direction_pnts = self._get_direction_points(fromDirPnt)
        for i, dir_pnt in enumerate(direction_pnts):
            next_pnt = pnt + dir_pnt
            newNode.dirNeighbours[i] = None

            if not self.is_point_valid(next_pnt):
                continue

            if self._is_movement_prohibited(pnt, next_pnt):
                continue

            newNode.dirNeighbours[i] = \
                self._update_neighbours(next_pnt, dir_pnt)

        if all(v is None for v in newNode.dirNeighbours):
            # No return as we start new stream back
            self._update_neighbours(pnt, fromDirPnt.invert_direction())

        return newNode

    def set_target(self, point):
        min_dist = 1e9
        nearest_pnt = None

        for _pnt in self.nodes:
            dist = self._calc_distance(_pnt, point)
            if dist < min_dist:
                min_dist = dist
                nearest_pnt = _pnt

        if nearest_pnt is None:
            raise Exception('Failed to find nearest node')

        self._target_node = self.nodes[nearest_pnt]

        self.logger.debug('Target is set to: {}'.format(self._target_node))

    def set_state(self, pointdir):
        min_dist = 1e9
        nearest_pntdir = None

        for _pntdir in self.directed_nodes:
            dist = self._calc_distance(_pntdir, pointdir)
            if dist < min_dist and _pntdir.d == pointdir.d:
                min_dist = dist
                nearest_pntdir = _pntdir

        if nearest_pntdir is None:
            raise Exception('Failed to find nearest node')

        self._current_node = self.directed_nodes[nearest_pntdir]

        self.logger.debug('State is set to: {}'.format(self._current_node))

    def next_local_target(self):
        if len(self._current_path) == 0:
            self._local_target_node = None
            return False

        self._current_node = self._current_path[0]
        self._update_path()

        return True

    def get_local_target(self):
        return self._local_target_node

    def set_limitation(self, limits):
        """Set limitations as 0/1 (1 - limited, 0 - free)

        Arguments:
            limits {list} -- [left, forward, right]
        """
        if self._local_target_node is None:
            self.logger.warn("self._local_target_node is None")
            return

        if self._local_target_node.is_turn():
            self.logger.warn("self._local_target_node.is_turn")
            return None

        # Because it is [r, f, l] inside - swap for better understanding
        self._local_target_node.dir_limitations[0] = limits[2]
        self._local_target_node.dir_limitations[1] = limits[1]
        self._local_target_node.dir_limitations[2] = limits[0]

    def _update_path(self):
        if self._target_node is None:
            raise Exception('Solver / Target not set')

        if self._current_node is None:
            raise Exception('Solver / State not set')

        if self._current_node.idx == self._target_node.idx:
            # Early ending - already on target
            self._current_path = []
            self._local_target_node = None
            return self._current_path

        self.logger.debug('Attempt to find path from node: {}'.format(self._current_node))

        frontier = PriorityQueue()
        frontier.put((0, self._current_node))

        cost_so_far = {}
        cost_so_far[self._current_node] = 0

        cameFrom = {}
        cameFrom[self._current_node] = None

        while not frontier.empty():
            _, cur_node = frontier.get()
            self.logger.debug('Next node: {}'.format(cur_node))
            self.logger.debug('  Neigbours: {} / {}'.format(cur_node.dirNeighbours, cur_node.dir_limitations))

            for idx, way_node in enumerate(cur_node.dirNeighbours):
                # No way or limited
                if way_node is None or cur_node.dir_limitations[idx] != 0:
                    continue

                if way_node.idx == self._target_node.idx:
                    # print('Got it!')
                    path = [self._target_node]

                    node = cur_node
                    while True:
                        if cameFrom[node] is None:
                            break

                        path.append(node)
                        node = cameFrom[node]

                    path.reverse()

                    self._current_path = path
                    self._local_target_node = self._current_path[0]
                    return path

                new_cost = get_manhattan_dist(
                    cur_node, way_node) + cost_so_far[cur_node]
                self.logger.debug('Neighbour found: {} / g = {}'.format(way_node, new_cost))

                if way_node not in cost_so_far or new_cost < cost_so_far[way_node]:

                    cost_so_far[way_node] = new_cost

                    prio_f = new_cost + \
                        self.calc_heuristic(way_node, self._target_node)
                    # print('    goes in frontier f = {}'.format(prio_f))
                    frontier.put((prio_f, way_node))

                    cameFrom[way_node] = cur_node

        # No path was found
        self._current_path = []
        self.logger.debug('Path not found')
        raise Exception('Solver / No path found')

    def get_path(self):
        self._update_path()
        return self._current_path


    @staticmethod
    def _angle_2_fromdir(ext_angle):
        angle = Maze._normalize_angle(ext_angle)

        angle_idx = round(angle / 90.)
        if angle_idx > 3:
            angle_idx = 0

        result_fromdir = None

        for fromdir_ltr in g_direction_angle:
            if int(angle_idx*90) == g_direction_angle[fromdir_ltr]:
                result_fromdir = fromdir_ltr

        if result_fromdir is None:
            raise Exception('Failed to get fromdir info')

        return result_fromdir

    @staticmethod
    def _normalize_angle(angle_deg):
        angle = int(angle_deg)
        angle = angle % 360
        angle = (angle + 360) % 360

        return angle

    def _get_direction_points(self, fromDirPnt):
        return [MazePoint(fromDirPnt.y, -fromDirPnt.x), fromDirPnt, MazePoint(-fromDirPnt.y, fromDirPnt.x)]

    def _calc_distance(self, pnt1, pnt2):
        return np.linalg.norm(pnt1.as_array()-pnt2.as_array())

    def calc_heuristic(self, next_node, target_node):
        return get_manhattan_dist(next_node, target_node)

    def is_element_vacant(self, elem):
        if elem == 0 or elem == 1 or elem == 2:
            return True

        return False

    def is_point_valid(self, p):
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

        return self.maze_array[self.height-p.y-1][p.x]

    ### RENDERING ###
    def _get_render_cell_size(self):
        return np.array([
            60,  # self.screen.get_width() / self.width,
            60  # self.screen.get_height() / self.height
        ])

    def render_maze(self):
        if not pygame_imported:
            return

        cell_colors = (255, 255, 255), (0, 255, 0), (128, 128, 255)

        cell_margin = 2
        cell_sz = self._get_render_cell_size()
        adjusted_sz = cell_sz - cell_margin

        if not hasattr(self, 'screen'):
            self.screen = pygame.display.set_mode(
                (cell_sz[0]*self.width, cell_sz[1]*self.height)
            )

        self.screen.fill((0, 0, 0))

        def render_get_cell_rect(pnt):
            x, y = pnt
            y = self.height - 1 - y
            return pygame.Rect(x * cell_sz[0] + cell_margin / 2,
                               y * cell_sz[1] + cell_margin / 2,
                               *adjusted_sz)

        font = pygame.font.Font(pygame.font.get_default_font(), 12)

        # White for edges
        for pnt in self.edges:
            coord = pnt.as_array()
            self.screen.fill(cell_colors[0], render_get_cell_rect(coord))

        # Green for nodes
        for pnt in self.nodes:
            coord = pnt.as_array()
            rect = render_get_cell_rect(coord)

            # if self.nodes[pnt] == self.start_node or self.nodes[pnt] == self.end_node:
            #    self.screen.fill(cell_colors[2], rect)
            # else:
            self.screen.fill(cell_colors[1], rect)

            text = font.render("{}".format(
                self.nodes[pnt].idx), True, (0, 0, 0))
            text_rect = text.get_rect()
            text_rect.center = rect.center

            self.screen.blit(text, text_rect)

        pygame.display.update()

        # Render possible directions


if __name__ == "__main__":
    if pygame_imported:
        pygame.init()

    ## ROS - 2 meters cell, x goes up, y goes left
    ## Maze - normalized cells, x goes right, y goes up

    ## These functions must be realized by yourself
    ## local TF -> ROS: (x1, y1) = (2*y, -2*x)
    ## ROS -> local TF: (x, y) = (-y1/2, x1/2)
    def ros2maze_point(ext_point):
        return MazePoint(-ext_point[1]/2., ext_point[0]/2.)   

    # logging.basicConfig(level=logging.INFO)
    logging.basicConfig(level=logging.DEBUG)

    structure = [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 8, 0, 8, 0, 8, 0],
        [0, 8, 0, 8, 0, 0, 0],
        [0, 0, 0, 0, 0, 8, 8],
        [0, 8, 0, 8, 0, 8, 8],
        [0, 8, 0, 8, 0, 0, 0],
        [0, 8, 0, 0, 0, 8, 8],
    ]

    edge_walls = [
        [MazePoint(0, 3), MazePoint(1, 3)]
    ]

    maze = Maze(structure, edge_walls=edge_walls)
    maze.render_maze()
    maze.set_target(MazePoint(6.5, 1))

    ## (0,0) point is lower-left corner
    ## Sample [y = 0.25] just for representation
    # initial_pose = ros2maze_pointdir([0, 0.25], 0)
    ## Change to new direction
    maze_initial_pnt = ros2maze_point([6.2*2, -1.2*2])
    initial_pose = MazePointDir(maze_initial_pnt.x, maze_initial_pnt.y, InitialDirection.LEFT)
    print('Initial: {}'.format(initial_pose))
    maze.set_state(initial_pose)

    while True:
        path = maze.get_path()
        if len(path) < 1:
            break

        logging.info('Path:')
        for node in path:
            logging.info('  {}'.format(node))

        # Test that limitation not work on turns
        if maze.get_local_target().idx == 1:
            maze.set_limitation([1, 0, 1])
            logging.info('set limitation: {}'.format([1, 0, 1]))
        elif maze.get_local_target().idx == 5:
            maze.set_limitation([1, 0, 0])
            logging.info('set limitation: {}'.format([1, 0, 0]))
        elif maze.get_local_target().idx == 4:
            maze.set_limitation([1, 0, 1])
            logging.info('set limitation: {}'.format([1, 0, 1]))

        maze.next_local_target()

        time.sleep(3)

    time.sleep(10)

    logging.info('Done')
