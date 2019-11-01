#!/usr/bin/env python3

"""
This create gui that allow to create json and sdf files.
"""

from gui import *

if __name__=="__main__":
    if len(sys.argv) == 6:
        defaultJsonPath = sys.argv[5]
    else:
        defaultJsonPath = ""
    if len(sys.argv) >= 5:
        cellsSize = [float(sys.argv[3]), float(sys.argv[4])]
    else:
        cellsSize = [2, 2]
    if len(sys.argv) >= 3:
        cellsAmount = [int(sys.argv[1]), int(sys.argv[2])]
    else:
        cellsAmount = [9, 9]
    if len(sys.argv) > 6:
        print("Error: too many arguments!")
    else:
        app = QApplication(sys.argv)
        window = MainWindow()
        Map.Init(cellsAmount, cellsSize)
        app.exec_()

