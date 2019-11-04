#!/usr/bin/env python3

"""
This sript creates gui that allow to create json and sdf files.
"""

from gui import *
from data_structures import *

if __name__=="__main__":
    DEFAULT_PATH = None
    DEFAULT_CELLS_SIZE = Size2D(2, 2)
    DEFAULT_CELLS_AMOUNT = Size2D(9, 9)

    if len(sys.argv) == 6:
        defaultJsonPath = sys.argv[5]
    else:
        defaultJsonPath = DEFAULT_PATH

    if len(sys.argv) >= 5:
        cellsSize = Size2D(float(sys.argv[3]), float(sys.argv[4]))
    else:
        cellsSize = DEFAULT_CELLS_SIZE

    if len(sys.argv) >= 3:
        cellsAmount = Size2D(int(sys.argv[1]), int(sys.argv[2]))
    else:
        cellsAmount = DEFAULT_CELLS_AMOUNT

    if len(sys.argv) > 6:
        print("Error: too many arguments!")
    else:
        app = QApplication(sys.argv)
        Map.Init(cellsAmount, cellsSize)
        window = MainWindow(defaultJsonPath)
        app.exec_()

