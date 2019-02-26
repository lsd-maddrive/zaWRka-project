import numpy as np
import pygame
from pygame.locals import *
import time

from maze import *
from car_state import *

import gui

# local TF -> ROS: (x1, y1) = (2*y, -2*x)
# ROS -> local TF: (x, y) = (-y1/2, x1/2)

def main(args=None):

    pygame.init()

    structure = [[0, 0, 0, 0, 0, 0, 0],
                 [0, 8, 0, 8, 0, 8, 0],
                 [0, 8, 0, 8, 0, 0, 0],
                 [0, 0, 0, 0, 0, 8, 8],
                 [0, 8, 0, 8, 0, 8, 8],
                 [0, 8, 0, 8, 0, 0, 2],
                 [1, 8, 0, 0, 0, 8, 8]]
    structure = np.array(structure, np.uint8)
    maze = Maze(structure)

    maze.render_maze()
    maze.nextPreprocessing()

    car = CarState(maze)
    car.renderCarPosition()

    while True:

        maze.render_maze()

        if not car.solveMazeAStar():
            gui.notifyUser('Failed to find path =(')
            print('Failed to find path...')
            break

        tNode = car.getNextTarget()
        # print('Next node: {}'.format(tNode))

        car.move2Target(tNode)

        if car.isEndReached():
            gui.notifyUser('Woohoo, we did it!')
            print('Woohoo, we did it!')
            break

        # Update display
        car.renderPath()
        car.renderCarPosition()
        print('Moved to {}'.format(tNode))
        
        if car.updateSignsInfo() < 0:
            break

        time.sleep(0.5)

        # Pygame checks
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == 32: # Space
                    exit()

    pygame.quit()


if __name__ == '__main__':
    main()
