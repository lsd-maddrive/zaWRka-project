
from .maze import *
import math as m

import numpy as np
from queue import PriorityQueue
import pygame

from . import gui

class CarState:
    def __init__(self, maze):
        self.maze = maze

        self.startNode = maze.start_node
        self.targetNode = maze.end_node

        self.cNode = self.startNode
        self.cameFrom = None

    def solveMazeAStar(self):
        
        print('Attempt to find path from node: {}'.format(self.cNode))

        frontier = PriorityQueue()
        frontier.put((0, self.cNode))

        cost_so_far = {}
        cost_so_far[self.cNode] = 0

        cameFrom = {}
        cameFrom[self.cNode] = None

        while not frontier.empty():
            _, cNode = frontier.get()
            print('Next node: {}'.format(cNode))

            currentWays = cNode.dirNeighbours
            for way in currentWays:
                if way is None:
                    continue

                if way == self.targetNode:
                    print('Got it!')
                    cameFrom[self.targetNode] = cNode
                    self.cameFrom = cameFrom
                    return True

                new_cost = getManhattanDistance(cNode, way) + cost_so_far[cNode]
                print('Neighbour found: {} / g = {}'.format(way, new_cost))

                if way not in cost_so_far or new_cost < cost_so_far[way]:
                    
                    cost_so_far[way] = new_cost

                    prio_f = new_cost + self.maze.calcHeuristic(way)
                    print('    goes in frontier f = {}'.format(prio_f))
                    frontier.put((prio_f, way))

                    cameFrom[way] = cNode

        return False


    def getNextTarget(self):
        pNode = None
        cNode = self.targetNode
        while 1:
            pNode = self.cameFrom[cNode]

            if pNode == self.cNode:
                return cNode

            cNode = pNode

    def isEndReached(self):
        return self.cNode == self.targetNode

    def move2Target(self, tNode):
        self.cNode = tNode
        print('Moved to node {}'.format(self.cNode))
        self.cNode.show_info()

    def updateSignsInfo(self):
        if self.cNode.signChecked:
            return 0

        # Test if node is just turn
        waysCnt = len([i for i, j in enumerate(self.cNode.dirNeighbours) if j is not None])

        if waysCnt == 1 and not self.cNode.signChecked:
            self.cNode.signChecked = True
            return 0

        signChoice = gui.requestSign()
        print('User chose {}'.format(signChoice))
        if signChoice < 0:
            return -1

        # Clean prohibites directions
        nodeRemoval = gui.signRemoveDirs[signChoice]
        for remIdx in nodeRemoval:
            self.cNode.dirNeighbours[remIdx] = None

        self.cNode.signChecked = True

        return 0

    def renderPath(self):
        cellWidth = self.maze.render_getCellWidth()

        rNode = self.targetNode
        while 1:
            nodeCntr = rNode.coord.get_array()
            nodeCntr = self.maze.render_Map2Canvas(nodeCntr)

            pygame.draw.circle(self.maze.screen, (0,0,255), nodeCntr, int(cellWidth/5))

            rNode = self.cameFrom[rNode]

            if rNode == self.cNode:
                return


    def renderCarPosition(self):

        cellWidth = self.maze.render_getCellWidth()

        carCenter = self.cNode.coord.get_array()
        carCenter = self.maze.render_Map2Canvas(carCenter)

        carDir = self.cNode.dirLetr

        carHalfLength = cellWidth/2.5
        carHalfWidth = cellWidth/3

        if carDir == '-':
            pygame.draw.circle(self.maze.screen, (0,0,0), carCenter, int(cellWidth/3))
        else:
            if carDir == 'U':
                polygon = [(carCenter[0], carCenter[1] + carHalfLength),
                           (carCenter [0] + carHalfWidth, carCenter[1] - carHalfLength),
                           (carCenter[0] - carHalfWidth, carCenter[1] - carHalfLength)]

            elif carDir == 'D':
                polygon = [(carCenter[0], carCenter[1] - carHalfLength),
                           (carCenter[0] + carHalfWidth, carCenter[1] + carHalfLength),
                           (carCenter[0] - carHalfWidth, carCenter[1] + carHalfLength)]
                
            
            elif carDir == 'L':
                polygon = [(carCenter[0] + carHalfLength, carCenter[1]),
                           (carCenter[0] - carHalfLength, carCenter[1] + carHalfWidth),
                           (carCenter[0] - carHalfLength, carCenter[1] - carHalfWidth)]

            elif carDir == 'R':
                polygon = [(carCenter[0] - carHalfLength, carCenter[1]),
                           (carCenter[0] + carHalfLength, carCenter[1] + carHalfWidth),
                           (carCenter[0] + carHalfLength, carCenter[1] - carHalfWidth)]

            pygame.draw.polygon(self.maze.screen, (0,0,0), polygon)

        pygame.display.update()
