#!/usr/bin/env python3

"""
This sript creates gui that allow to create json and sdf files.
"""

from gui import *
from data_structures import *

import argparse

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Map creation tool')
    parser.add_argument('--jdir', 
                        type=str, 
                        help='Default JSON dir',
                        default=None)
    parser.add_argument('--size', 
                        help='Size of map [measured in cells] (w x h)',
                        default='9x9')
    parser.add_argument('--cell', 
                        help='Size of cell in map',
                        default='1x1')

    args = vars(parser.parse_args())

    map_w, map_h = args['size'].split('x')
    cell_w, cell_h = args['cell'].split('x')
    defaultJsonPath = args['jdir']

    cellsSize = Size2D(float(cell_w), float(cell_h))
    cellsAmount = Size2D(int(map_w), int(map_h))

    app = QApplication(sys.argv)
    Map.Init(cellsAmount, cellsSize)
    window = MainWindow(defaultJsonPath)
    app.exec_()

