#!/usr/bin/env python3
import sys, random
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, \
    QLabel, QDialog, QFileDialog
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QIcon, QImage
from PyQt5.QtCore import Qt, QSize
from enum import Enum
from json_converter import *
from json_constants import *

# ************************** Constants and enums *****************************
class Mode(Enum):
    NO_MODE = int(-1)
    MAP_SIZE = int(0)
    CELL_SIZE = int(1)
    START = int(2)
    END = int(3)
    BOXES = int(4)
    WALLS = int(5)
    SIGNS = int(6)
    LIGHTS = int(7)

class CollorCode(Enum):
    WHITE = str("FFFFFF")
    BLUE = str("0000FF")
    GREEN = str("00FF00")
    RED = str("FF0000")



# ***************************** Main window *********************************
class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.setGeometry(300, 300, 710, 320)
        self.setWindowTitle('World creator v.2')
        self.show()

        Map.Init([18, 18])
        self.__features = ControlPanel.Init(self)


    def mousePressEvent(self, e):
        if ControlPanel.mode is not Mode.NO_MODE:
            self.__features[ControlPanel.mode.value].processMousePressing(e)
        else:
            print("Warning: you should choose mode")
        self.update()


    def paintEvent(self, event=None):
        startPose = self.__features[Mode.START.value].start
        qp = QPainter()
        qp.begin(self)
        self.drawPoints(qp)
        self.drawMap(qp)
        for feature in self.__features:
            feature.processPaint(qp)
        qp.end()


    def drawPoints(self, qp):
        """ 
        @brief Draw window background with random points cloud
        """
        qp.setPen(Qt.red)
        size = self.size()
        for i in range(1000):
            x = random.randint(1, size.width()-1)
            y = random.randint(1, size.height()-1)
            qp.drawPoint(x, y)

    @staticmethod
    def drawLine(qp, firstPoseOnWindow, secondPoseOnWindow):
        """ 
        @brief Draw line
        @note it uses real window coordinates 
        """
        pen = QPen(Qt.black, 3, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawLine(*firstPoseOnWindow, *secondPoseOnWindow)

    @staticmethod
    def drawRectangle(qp, centerPosition, size, collor=QColor(255, 0, 0)):
        """ 
        @brief Draw rectangle
        @note it uses real window coordinates 
        """
        brush = QBrush(collor)
        qp.setBrush(brush)
        left = centerPosition[0] - ControlPanel.CellsSize[0]
        top = centerPosition[1] - ControlPanel.CellsSize[1]
        qp.drawRect(left, top, ControlPanel.CellsSize[0], ControlPanel.CellsSize[1])

    @staticmethod
    def drawImg(qp, centerPosition, imgPath = ImagesPaths.STOP):
        """ 
        @brief Draw a sign
        @note it uses real window coordinates 
        """
        halfCellsSizes = [ControlPanel.CellsSize[0]/2, ControlPanel.CellsSize[1]/2]
        left = centerPosition[0] - halfCellsSizes[0]
        top = centerPosition[1] - halfCellsSizes[1]
        img = QImage(imgPath).scaled(QSize(halfCellsSizes[0], halfCellsSizes[1]))
        qp.drawImage(left, top, img)


    def drawMap(self, qp):
        """ 
        @brief Draw map with cells
        """
        thinPen = QPen(Qt.black, 0.5, Qt.SolidLine)
        qp.setPen(thinPen)

        windowWidth = self.frameGeometry().width()
        windowHeight = self.frameGeometry().height()

        ControlPanel.Right = int(0.70 * windowWidth)
        ControlPanel.Left = int(0.05 * windowWidth)
        ControlPanel.Bot = int(0.9 * windowHeight)
        ControlPanel.Top = int(0.05 * windowHeight)

        # Table sizes must divided on cell size amount without remainder
        ControlPanel.Left -= (ControlPanel.Left - ControlPanel.Right) % Map.CELLS_AMOUNT[0]
        ControlPanel.Top -= (ControlPanel.Top - ControlPanel.Bot) % Map.CELLS_AMOUNT[1]

        self.tableWidth = ControlPanel.Right - ControlPanel.Left
        self.tableHeight = ControlPanel.Bot - ControlPanel.Top
        ControlPanel.CellsSize = [ int(self.tableWidth/Map.CELLS_AMOUNT[0]), 
                                   int(self.tableHeight/Map.CELLS_AMOUNT[1]) ]

        for row in range(ControlPanel.Top, ControlPanel.Bot + 1, ControlPanel.CellsSize[1]):
            qp.drawLine(ControlPanel.Left, row, ControlPanel.Right, row)
        for col in range(ControlPanel.Left, ControlPanel.Right + 1, ControlPanel.CellsSize[0]):
            qp.drawLine(col, ControlPanel.Top, col, ControlPanel.Bot)



# ********************************* Map *************************************
class Map:
    """
    Static class with map settings
    """
    @staticmethod
    def Init(mapSize):
        Map.SIZE = mapSize
        Map.CELLS_SIZE_IN_METERS = [2, 2]
        Map.CELLS_AMOUNT = [ int(Map.SIZE[0]/Map.CELLS_SIZE_IN_METERS[0]), 
                              int(Map.SIZE[1]/Map.CELLS_SIZE_IN_METERS[1]) ]



# ***************************** Control panel ********************************
class ControlPanel():
    @staticmethod
    def Init(window):
        ControlPanel.mode = Mode.NO_MODE
        ControlPanel.window = window

        ControlPanel.Right = int()
        ControlPanel.Left = int()
        ControlPanel.Bot = int()
        ControlPanel.Top = int()
        ControlPanel.CellsSize = list()

        window.layout = QGridLayout()
        window.setLayout(window.layout)
        label = QLabel(' ', window)
        window.layout.addWidget(label, 0, 0)
        window.layout.setSpacing(1)

        ControlPanel.features = list()
        ControlPanel.features.append(Feature('1. Choose map size', False))
        ControlPanel.features.append(Feature('2. Choose cells size', False))
        ControlPanel.features.append(StartPosition('3. Choose start pose'))
        ControlPanel.features.append(Feature('4. Choose end pose', False))
        ControlPanel.features.append(Feature('5. Create boxes', False))
        ControlPanel.features.append(Wall('6. Create walls'))
        ControlPanel.features.append(Sign('7. Create signs'))
        ControlPanel.features.append(Feature('8. Create lights', False))
        ControlPanel.features.append(LoadJson('Load json'))
        ControlPanel.features.append(GenerateJson('Generate json'))
        ControlPanel.features.append(CreateSdf('Create sdf world from json'))

        for i in range(0, len(ControlPanel.features)):
            window.layout.addWidget(ControlPanel.features[i].button, i + 1, 1)
        window.layout.addWidget(ControlPanel.features[8].button, 10, 1)
        window.layout.addWidget(ControlPanel.features[9].button, 12, 1)
        window.layout.addWidget(ControlPanel.features[10].button, 13, 1)

        window.layout.addWidget(QLabel('To create the world:', window), 0, 1)
        window.layout.addWidget(QLabel('Or use these features:', window), 9, 1)
        window.layout.addWidget(QLabel('Then press buttons below:',window), 11, 1)

        return ControlPanel.features

    @staticmethod
    def SetMode(mode):
        """
        @brief set mode from class Mode(enum)
        """
        try:
            print(mode)
            for i in range(0, 9):
                ControlPanel.setButtonCollor(ControlPanel.features[i].button, CollorCode.WHITE)
            ControlPanel.setButtonCollor(ControlPanel.features[mode.value].button, CollorCode.RED)
            ControlPanel.mode = mode
        except:
            print("Error: mode must be Enum")

    @staticmethod
    def createButton(name):
        but = QPushButton(name, ControlPanel.window)
        but.setFixedSize(QSize(200, 25))
        return but

    @staticmethod
    def setButtonCollor(but, collor = CollorCode.WHITE):
        but.setStyleSheet("QPushButton {background-color: #" + \
                          collor.value + "}")
    @staticmethod
    def PrintObjects():
        print("Objects are:")
        print("start: ", ControlPanel.features[Mode.START.value].start)
        print("finish: ", " ")
        print("size: ", Map.SIZE)
        print("boxes: ", " ")
        print("Walls: ", ControlPanel.features[Mode.WALLS.value].Walls)
        print("Signs: ", ControlPanel.features[Mode.SIGNS.value].Signs)


# ******************************* Features ***********************************
class Feature():
    """ @brief Interface for button features """
    def __init__(self, buttonText, isItEnable=True):
        """ @brief Init button, you shouldn't override this constuctor """
        self.button = ControlPanel.createButton(buttonText)
        self.button.pressed.connect(self.processButtonPressing)
        self.button.setEnabled(isItEnable)
        self._initOtherParameters()
    def _initOtherParameters(self):
        """ @brief Additional initializations add here """
        pass
    def processButtonPressing(self):
        pass
    def processMousePressing(self):
        pass
    def processPaint(self, qp):
        pass

    @staticmethod
    def _mousePoseToCellIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = [int()] * 2
        for axe in range(0, 2):
            node[axe] = int(tablePose[axe] / ControlPanel.CellsSize[axe])
            if node[axe] > (Map.CELLS_AMOUNT[axe] + 1) or (node[axe] < 0):
                return None
        return node

    @staticmethod
    def _mousePoseToNodeIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = [int()] * 2
        for axe in range(0, 2):
            node[axe] = int(tablePose[axe] / ControlPanel.CellsSize[axe])
            # Line below is needed because of unexpected work of division of
            # negative nubmers  
            if tablePose[axe] < 0: node[axe] -= 1 
            if tablePose[axe] % ControlPanel.CellsSize[axe] > ControlPanel.CellsSize[axe] / 2:
                node[axe] += 1
        return node

    @staticmethod
    def _mousePoseToEdgeIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = [int()] * 2
        for axe in range(0, 2):
            node[axe] = int(tablePose[axe] / ControlPanel.CellsSize[axe])
            fourthNode = ControlPanel.CellsSize[axe]/4
            remnant = tablePose[axe] % ControlPanel.CellsSize[axe]
            if (remnant >= fourthNode) and (remnant <= 3*fourthNode):
                node[axe] += 0.5
            elif remnant > 3*fourthNode:
                node[axe] += 1
        if(((node[0] % 1) is 0) and ((node[1] % 1) is not 0)) or \
          (((node[0] % 1) is not 0) and ((node[1] % 1) is 0)):
            return node
        return None

    @staticmethod
    def _mousePoseToPositionIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        pose = [int()] * 2
        for axe in range(0, 2):
            meterSize = ControlPanel.CellsSize[axe] / Map.CELLS_SIZE_IN_METERS[axe]
            pose[axe] = int(tablePose[axe] / meterSize)
            # Line below is needed because of division of negative nubmers  
            if tablePose[axe] < 0: pose[axe] -= 1 
        return pose

    @staticmethod
    def _cellIndexesToMousePose(cellIndexes):
        return [ControlPanel.Left + (cellIndexes[0] + 1) * ControlPanel.CellsSize[0],
                ControlPanel.Top + (cellIndexes[1] + 1) * ControlPanel.CellsSize[1]]
    
    @staticmethod
    def _positionIndexesToMousePose(poseIndexes):
        return [ControlPanel.Left + (poseIndexes[0] + 1) * ControlPanel.CellsSize[0]/2,
                ControlPanel.Top + (poseIndexes[1] + 1) * ControlPanel.CellsSize[1]/2]

    @staticmethod
    def _nodeIndexesToMousePose(nodeIndexes):
        return [ ControlPanel.Left + nodeIndexes[0] * ControlPanel.CellsSize[0],
                 ControlPanel.Top + nodeIndexes[1] * ControlPanel.CellsSize[1] ]



class StartPosition(Feature):
    def _initOtherParameters(self):
        self.start = None
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.START)
    def processMousePressing(self, e):
        self.start = Feature._mousePoseToCellIndexes(e.pos().x(), e.pos().y())
        print("start pose was setted using mouse: " + str(self.start))
    def processPaint(self, qp):
        if self.start is not None:
            self.__draw(qp, self.start)
    @staticmethod
    def __draw(qp, cellIndexes):
        centerPos = Feature._cellIndexesToMousePose(cellIndexes)
        MainWindow.drawRectangle(qp, centerPos, ControlPanel.CellsSize)


class Wall(Feature):
    def _initOtherParameters(self):
        self.__lastClickNumber = 0
        self.__pressedFirstNode = None
        self.__pressedSecondNode = None
        self.Walls = list()
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.WALLS)
    def processMousePressing(self, e):
        pos = e.pos()
        if(e.button() == 1):
            pos = Feature._mousePoseToNodeIndexes(pos.x(), pos.y())
            if self.__lastClickNumber is 1:
                self.__lastClickNumber = 2
                self.__pressedSecondNode = pos
                self.__addWall([self.__pressedFirstNode, self.__pressedSecondNode])
            else:
                self.__lastClickNumber = 1
                self.__pressedFirstNode = pos
        else:
            pos = Feature._mousePoseToEdgeIndexes(pos.x(), pos.y())
            self.__deleteWall(pos)
            self.__lastClickNumber = 0
    def processPaint(self, qp):
        for wall in self.Walls:
            Wall.__draw(qp, wall[0], wall[1])
    def __addWall(self, nodesIndexes):
        if self.__isWallPossible(nodesIndexes) is True:
            print("Wall was added: " + str(nodesIndexes))
            self.Walls.append(nodesIndexes)
    def __isWallPossible(self, nodesIndexes):
        if self.__isWallOutOfRange(nodesIndexes) is True:
            print("Warning: wall out of map: " + str(nodesIndexes))
        elif self.__isThisWallPoint(nodesIndexes) is True:
            print("Warning: wall can't be point: " + str(nodesIndexes))
        elif self.__isThisWallDiagonal(nodesIndexes) is True:
            print("Warning: wall can't be diagonal: " + str(nodesIndexes))
        elif self.__isThereConflictBetweenWalls(nodesIndexes) is True:
            print("Warning: there is conflict between existing walls \
            and this: " + str(nodesIndexes))
        else:
            return True
        return False
    def __isThisWallPoint(self, nodesIndexes):
        return nodesIndexes[0] == nodesIndexes[1]
    def __isWallOutOfRange(self, nodesIndexes):
        return  nodesIndexes[0][0] > Map.CELLS_AMOUNT[0] or \
                nodesIndexes[0][0] < 0 or \
                nodesIndexes[1][0] > Map.CELLS_AMOUNT[0] or \
                nodesIndexes[1][0] < 0 or \
                nodesIndexes[0][1] > Map.CELLS_AMOUNT[1] or \
                nodesIndexes[0][1] < 0 or \
                nodesIndexes[1][1] > Map.CELLS_AMOUNT[1] or \
                nodesIndexes[1][1] < 0
    def __isThisWallDiagonal(self, nodesIndexes):
        return ((self.__isThisWallVertical(nodesIndexes) is False) and \
                (self.__isThisWallHorizontal(nodesIndexes) is False))
    def __isThisWallVertical(self, nodesIndexes):
        return nodesIndexes[0][0] == nodesIndexes[1][0]
    def __isThisWallHorizontal(self, nodesIndexes):
        return nodesIndexes[0][1] == nodesIndexes[1][1]
    def __isThereConflictBetweenWalls(self, nodesIndexes):
        return False
    def __deleteWall(self, pos):
        print("wall with pose: ", pos)
        try:
            isVerticalFound = False
            isHorizontalFound = False
            for wall in self.Walls:
                if((wall[0][0] is pos[0]) and (wall[1][0] is pos[0])):
                    if(wall[0][1] <= pos[1] and wall[1][1] >= pos[1]) or \
                      (wall[1][1] <= pos[1] and wall[0][1] >= pos[1]):
                        isVerticalFound = True
                        break
                if(wall[0][1] is pos[1]) and (wall[1][1] is pos[1]):
                    if(wall[0][0] <= pos[0] and wall[1][0] >= pos[0]) or \
                      (wall[1][0] <= pos[0] and wall[0][0] >= pos[0]):
                        isHorizontalFound = True
                        break
            if isVerticalFound == True:
                print("delete vertical: " + str(wall) + " and " + str(pos))
                self.Walls.remove(wall)
            elif isHorizontalFound == True:
                print("delete horizontal: " + str(wall) + " and " + str(pos))
                self.Walls.remove(wall)
            ControlPanel.window.update()
        except:
            print("Warning: it's not wall")
    @staticmethod
    def __draw(qp, indexesOfNode1, indexesOfNode2):
        posOfNode1 = Feature._nodeIndexesToMousePose(indexesOfNode1)
        posOfNode2 = Feature._nodeIndexesToMousePose(indexesOfNode2)
        MainWindow.drawLine(qp, posOfNode1, posOfNode2)


class Sign(Feature):
    def _initOtherParameters(self):
        self.Signs = list()
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.SIGNS)
    def processMousePressing(self, e):
        pos = Feature._mousePoseToPositionIndexes(e.pos().x(), e.pos().y())
        self.signChoiceDialog = SignChoiceDialog(pos)
    def processPaint(self, qp):
        for sign in self.Signs:
            Sign.__draw(qp, sign[0], sign[1])
    @staticmethod
    def __draw(qp, poseIndexes, imgPath):
        centerPos = Feature._positionIndexesToMousePose(poseIndexes)
        MainWindow.drawImg(qp, centerPos, imgPath)

class SignChoiceDialog(QDialog):
    def __init__(self, pos, parent=None):
        super(SignChoiceDialog, self).__init__(parent)
        self.__pos = pos
        self.__window = ControlPanel.window
        self.__createDialog()
        self.__signs = ControlPanel.features[Mode.SIGNS.value].Signs
    def __createDialog(self):
        self.__dialog = QDialog()
        self.__dialog.label = QLabel("Choose the sign:")
        self.__dialog.buttons = list()
        info = list([ ["0. Empty", None],
                      ["1. Stop", ImagesPaths.STOP],
                      ["2. Forward", ImagesPaths.ONLY_FORWARD],
                      ["3. Left", ImagesPaths.ONLY_LEFT],
                      ["4. Right", ImagesPaths.ONLY_RIGHT],
                      ["5. Forward or left", ImagesPaths.FORWARD_OR_LEFT],
                      ["6. Forward or right", ImagesPaths.FORWARD_OR_RIGHT],])
        layout = QGridLayout()
        layout.addWidget(self.__dialog.label, 0, 0)
        self.__dialog.buttons.append(QPushButton(info[0][0]))
        callback = lambda: self.__deleteSign(self.__pos)
        self.__dialog.buttons[0].clicked.connect(callback)
        layout.addWidget(self.__dialog.buttons[0], 1, 0)
        for i in range(1, 7):
            self.__dialog.buttons.append(QPushButton(info[i][0]))
            self.__dialog.buttons[i].setIcon(QIcon(info[i][1]))
            self.__dialog.buttons[i].setIconSize(QSize(24, 24))
            callback = lambda this, row=i: self.__addSign(self.__pos,
                info[row][1])
            self.__dialog.buttons[i].clicked.connect(callback)
            layout.addWidget(self.__dialog.buttons[i], i+1, 0)
        self.__dialog.setLayout(layout)
        self.__dialog.show()
    def __addSign(self, poseIndexes, signImg):
        print("Sign {1} was added in {0}.".format(poseIndexes, signImg))
        self.__signs.append([poseIndexes, signImg])
        self.__dialog.close()
        self.__window.update()
    def __deleteSign(self, poseIndexes):
        for sign in self.__signs:
            if sign[0] == poseIndexes:
                print("Sign was deleted: " + str(poseIndexes))
                self.__signs.remove(sign)
        self.__dialog.close()
        self.__window.update()


class LoadJson(Feature):
    def processButtonPressing(self):
        FILE_TYPES = "Json Files (*.json)"
        filePath = QFileDialog.getOpenFileName(ControlPanel.window, "", "", FILE_TYPES)[0]
        objects = load_frontend_from_json(filePath)
        ControlPanel.features[Mode.START.value].start = objects[0]
        finish = objects[1]
        size = objects[2]
        boxes = objects[3]
        ControlPanel.features[Mode.WALLS.value].Walls = objects[4]
        ControlPanel.features[Mode.SIGNS.value].Signs = objects[5]
        print("Load json:")
        ControlPanel.PrintObjects()
        ControlPanel.window.update()


class GenerateJson(Feature):
    def processButtonPressing(self):
        start = ControlPanel.features[Mode.START.value].start
        self.__walls = ControlPanel.features[Mode.WALLS.value].Walls
        self.__signs = ControlPanel.features[Mode.SIGNS.value].Signs
        if start is not None:
            create_json_from_gui(start, Map.SIZE, None, self.__walls, self.__signs)
            print("Generate json:")
            ControlPanel.PrintObjects()
        else:
            print("Warning: firstly, you should set start position.")


class CreateSdf(Feature):
    def processButtonPressing(self):
        create_sdf_from_json()
