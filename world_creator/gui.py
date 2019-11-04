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
    FINISH = int(3)
    BOXES = int(4)
    WALLS = int(5)
    SIGNS = int(6)
    LIGHTS = int(7)
    LOAD_JSON = int(8)

class CollorCode(Enum):
    WHITE = str("FFFFFF")
    BLUE = str("0000FF")
    GREEN = str("00FF00")
    RED = str("FF0000")



# ***************************** Main window *********************************
class MainWindow(QWidget):
    def __init__(self, filePath):
        super().__init__()
        LEFT_EDGE_POS = 300
        TOP_EDGE_POS = 300
        WIDTH = 710
        HEIGHT = 300
        WINDOW_TITLE = 'World creator v.2'
        self.setGeometry(LEFT_EDGE_POS, TOP_EDGE_POS, WIDTH, HEIGHT)
        self.setWindowTitle(WINDOW_TITLE)
        self.show()
        self.__features = ControlPanel.Init(self)
        if filePath is not None:
            print("\nTry to load file {0}:".format(filePath))
            self.__features[Mode.LOAD_JSON.value].loadJson(filePath)
            print("")


    def mousePressEvent(self, e):
        if ControlPanel.mode is not Mode.NO_MODE:
            self.__features[ControlPanel.mode.value].processMousePressing(e)
        else:
            print("Warning: you should choose mode")
        self.update()


    def paintEvent(self, event=None):
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
        ControlPanel.Left -= (ControlPanel.Left - ControlPanel.Right) % Map.CELLS_AMOUNT.x
        ControlPanel.Top -= (ControlPanel.Top - ControlPanel.Bot) % Map.CELLS_AMOUNT.y

        self.tableWidth = ControlPanel.Right - ControlPanel.Left
        self.tableHeight = ControlPanel.Bot - ControlPanel.Top
        ControlPanel.CellsSize = [ int(self.tableWidth/Map.CELLS_AMOUNT.x), 
                                   int(self.tableHeight/Map.CELLS_AMOUNT.y) ]

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
    def Init(cellsAmount, cellsSize):
        Map.CELLS_AMOUNT = Size2D(cellsAmount)
        Map.CELLS_SIZE = Size2D(cellsSize)
        Map.SIZE = Size2D([ Map.CELLS_AMOUNT.x * Map.CELLS_SIZE.x,
                            Map.CELLS_AMOUNT.y * Map.CELLS_SIZE.y])
        print("Init world with cells_amount =", Map.CELLS_AMOUNT.getStringData(),
              "and cells_size =", Map.CELLS_SIZE.getStringData())



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
        ControlPanel.features.append(BaseGuiObject('1. Choose map size', False))
        ControlPanel.features.append(BaseGuiObject('2. Choose cells size', False))
        ControlPanel.features.append(GuiStart('3. Choose start pose'))
        ControlPanel.features.append(GuiFinish('4. Choose end pose', False))
        ControlPanel.features.append(BaseGuiObject('5. Create boxes', False))
        ControlPanel.features.append(GuiWalls('6. Create walls'))
        ControlPanel.features.append(Sign('7. Create signs'))
        ControlPanel.features.append(BaseGuiObject('8. Create lights', False))
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
        print("start: ", ControlPanel.features[Mode.START.value].start.getListData())
        print("finish: ", ControlPanel.features[Mode.FINISH.value].finish.getListData())
        print("cellsAmount: ", Map.CELLS_AMOUNT.getStringData())
        print("cellsSize: ", Map.CELLS_SIZE.getStringData())
        print("mapSize: ", Map.SIZE.getStringData())
        print("boxes: ", " ")
        print("Walls: ", ControlPanel.features[Mode.WALLS.value].walls.toString())
        print("Signs: ", ControlPanel.features[Mode.SIGNS.value].Signs)


# ***************************** BaseGuiObject ********************************
class BaseGuiObject():
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

    # for start
    @staticmethod
    def _mousePoseToCellIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = Point2D()
        node.x = int(tablePose[0] / ControlPanel.CellsSize[0])
        if node.x > (Map.CELLS_AMOUNT.x + 1) or (node.x < 0):
            return None
        node.y = int(tablePose[1] / ControlPanel.CellsSize[1])
        if node.y > (Map.CELLS_AMOUNT.y + 1) or (node.y < 0):
            return None

        return node

    # for wall
    @staticmethod
    def _mousePoseToNodeIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = [int()] * 2

        node[0] = int(tablePose[0] / ControlPanel.CellsSize[0])
        # Line below is needed because of negative nubmers division
        if tablePose[0] < 0: node[0] -= 1 
        if tablePose[0] % ControlPanel.CellsSize[0] > ControlPanel.CellsSize[0] / 2:
            node[0] += 1

        node[1] = int(tablePose[1] / ControlPanel.CellsSize[1])
        # Line below is needed because of negative nubmers division
        if tablePose[1] < 0: node[1] -= 1 
        if tablePose[1] % ControlPanel.CellsSize[1] > ControlPanel.CellsSize[1] / 2:
            node[1] += 1
        return node

    # for wall
    @staticmethod
    def _mousePoseToEdgeIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = [int()] * 2

        node[0] = int(tablePose[0] / ControlPanel.CellsSize[0])
        fourthNode = ControlPanel.CellsSize[0]/4
        remnant = tablePose[0] % ControlPanel.CellsSize[0]
        if (remnant >= fourthNode) and (remnant <= 3*fourthNode):
            node[0] += 0.5
        elif remnant > 3*fourthNode:
            node[0] += 1

        node[1] = int(tablePose[1] / ControlPanel.CellsSize[1])
        fourthNode = ControlPanel.CellsSize[1]/4
        remnant = tablePose[1] % ControlPanel.CellsSize[1]
        if (remnant >= fourthNode) and (remnant <= 3*fourthNode):
            node[1] += 0.5
        elif remnant > 3*fourthNode:
            node[1] += 1

        if(((node[0] % 1) is 0) and ((node[1] % 1) is not 0)) or \
          (((node[0] % 1) is not 0) and ((node[1] % 1) is 0)):
            return node
        return None

    # for sign
    @staticmethod
    def _mousePoseToPositionIndexes(point_x, point_y):
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        pose = [int()] * 2

        halfCellSize = ControlPanel.CellsSize[0] / 2
        pose[0] = int(tablePose[0] / halfCellSize)
        # Line below is needed because of division of negative nubmers  
        if tablePose[0] < 0: pose[0] -= 1 

        halfCellSize = ControlPanel.CellsSize[1] / 2
        pose[1] = int(tablePose[1] / halfCellSize)
        # Line below is needed because of division of negative nubmers  
        if tablePose[1] < 0: pose[1] -= 1 

        return pose

    # for start
    @staticmethod
    def _cellIndexesToMousePose(cellIndexes):
        return [ControlPanel.Left + (cellIndexes.x + 1) * ControlPanel.CellsSize[0],
                ControlPanel.Top + (cellIndexes.y + 1) * ControlPanel.CellsSize[1]]
    
    # for wall
    @staticmethod
    def _nodeIndexesToMousePose(nodeIndexes):
        return [ ControlPanel.Left + nodeIndexes.x * ControlPanel.CellsSize[0],
                 ControlPanel.Top + nodeIndexes.y * ControlPanel.CellsSize[1] ]

    # for sign
    @staticmethod
    def _positionIndexesToMousePose(poseIndexes):
        return [ControlPanel.Left + (poseIndexes[0] + 1) * ControlPanel.CellsSize[0]/2,
                ControlPanel.Top + (poseIndexes[1] + 1) * ControlPanel.CellsSize[1]/2]



class GuiStart(BaseGuiObject):
    def _initOtherParameters(self):
        self.start = Start()
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.START)
    def processMousePressing(self, e):
        start = BaseGuiObject._mousePoseToCellIndexes(e.pos().x(), e.pos().y())
        self.start = Start(start)
        print("start pose was setted using mouse:", self.start.getData().getStringData())
    def processPaint(self, qp):
        if self.start.getData() is not None:
            self.__draw(qp, self.start.getData())
    @staticmethod
    def __draw(qp, cellIndexes):
        centerPos = BaseGuiObject._cellIndexesToMousePose(cellIndexes)
        MainWindow.drawRectangle(qp, centerPos, ControlPanel.CellsSize)


class GuiFinish(BaseGuiObject):
    def _initOtherParameters(self):
        self.finish = Finish()

class GuiWalls(BaseGuiObject):
    def _initOtherParameters(self):
        self.__lastClickNumber = 0
        self.__pressedFirstNode = None
        self.__pressedSecondNode = None
        self.walls = Walls()
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.WALLS)
    def processMousePressing(self, e):
        pos = e.pos()
        if(e.button() == 1):
            pos = BaseGuiObject._mousePoseToNodeIndexes(pos.x(), pos.y())
            if self.__lastClickNumber is 1:
                self.__lastClickNumber = 2
                self.__pressedSecondNode = pos
                wall = Wall(Point2D(self.__pressedFirstNode),
                            Point2D(self.__pressedSecondNode))
                self.__addWall(wall)
            else:
                self.__lastClickNumber = 1
                self.__pressedFirstNode = pos
        else:
            pos = BaseGuiObject._mousePoseToEdgeIndexes(pos.x(), pos.y())
            self.__deleteWall(pos)
            self.__lastClickNumber = 0
    def processPaint(self, qp):
        for wall in self.walls:
            GuiWalls.__draw(qp, wall)
    def __addWall(self, wall):
        if self.__isWallPossible(wall) is True:
            print("Add object: wall with pose " + wall.getStringData())
            self.walls.add(wall)
    def __isWallPossible(self, wall):
        if self.__isWallOutOfRange(wall) is True:
            print("Warning: wall out of map: " + wall.getStringData())
        elif self.__isThisWallPoint(wall) is True:
            print("Warning: wall can't be point: " + wall.getStringData())
        elif self.__isThisWallDiagonal(wall) is True:
            print("Warning: wall can't be diagonal: " + wall.getStringData())
        elif self.__isThereConflictBetweenWalls(wall) is True:
            print("Warning: there is conflict between existing walls \
            and this: " + wall.getStringData())
        else:
            return True
        return False
    def __isThisWallPoint(self, wall):
        return wall.point1 == wall.point2
    def __isWallOutOfRange(self, wall):
        return  wall.point1.x > Map.CELLS_AMOUNT.x or \
                wall.point1.x < 0 or \
                wall.point2.x > Map.CELLS_AMOUNT.x or \
                wall.point2.x < 0 or \
                wall.point1.y > Map.CELLS_AMOUNT.y or \
                wall.point1.y < 0 or \
                wall.point2.y > Map.CELLS_AMOUNT.y or \
                wall.point2.y < 0
    def __isThisWallDiagonal(self, wall):
        return ((self.__isThisWallVertical(wall) is False) and \
                (self.__isThisWallHorizontal(wall) is False))
    def __isThisWallVertical(self, wall):
        return wall.point1.x == wall.point2.x
    def __isThisWallHorizontal(self, wall):
        return wall.point1.y == wall.point2.y
    def __isThereConflictBetweenWalls(self, wall):
        return False
    def __deleteWall(self, pos):
        if pos is None:
            print("Note: you should choose middle of cell edge")
            return
        isVerticalFound = False
        isHorizontalFound = False
        for wall in self.walls:
            if((wall.point1.x is pos[0]) and (wall.point2.x is pos[0])):
                if(wall.point1.y <= pos[1] and wall.point2.y >= pos[1]) or \
                  (wall.point2.y <= pos[1] and wall.point1.y >= pos[1]):
                    isVerticalFound = True
                    break
            if(wall.point1.y is pos[1]) and (wall.point2.y is pos[1]):
                if(wall.point1.x <= pos[0] and wall.point2.x >= pos[0]) or \
                  (wall.point2.x <= pos[0] and wall.point1.x >= pos[0]):
                    isHorizontalFound = True
                    break
        if isVerticalFound == True:
            wallType = "vertical"
            self.walls.remove(wall)
        elif isHorizontalFound == True:
            wallType = "horizontal"
            self.walls.remove(wall)
        else:
            print("Note: there is no wall here")
            return
        print("Delete object: {0} wall with pose {1}".format(wallType, pos))
        ControlPanel.window.update()

    @staticmethod
    def __draw(qp, wall):
        posOfNode1 = BaseGuiObject._nodeIndexesToMousePose(wall.point1)
        posOfNode2 = BaseGuiObject._nodeIndexesToMousePose(wall.point2)
        MainWindow.drawLine(qp, posOfNode1, posOfNode2)


class Sign(BaseGuiObject):
    def _initOtherParameters(self):
        self.Signs = list()
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.SIGNS)
    def processMousePressing(self, e):
        pos = BaseGuiObject._mousePoseToPositionIndexes(e.pos().x(), e.pos().y())
        self.signChoiceDialog = SignChoiceDialog(pos)
    def processPaint(self, qp):
        for sign in self.Signs:
            Sign.__draw(qp, sign[0], sign[1])
    @staticmethod
    def __draw(qp, poseIndexes, imgPath):
        centerPos = BaseGuiObject._positionIndexesToMousePose(poseIndexes)
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
        print("Add object: sign {1} with pose {0}.".format(poseIndexes, signImg))
        self.__signs.append([poseIndexes, signImg])
        self.__dialog.close()
        self.__window.update()
    def __deleteSign(self, poseIndexes):
        for sign in self.__signs:
            if sign[0] == poseIndexes:
                print("Delete object: sign with pose" + str(poseIndexes))
                self.__signs.remove(sign)
        self.__dialog.close()
        self.__window.update()


class LoadJson(BaseGuiObject):
    def processButtonPressing(self):
        print("\nLoad json:")
        FILE_TYPES = "Json Files (*.json)"
        filePath = QFileDialog.getOpenFileName(ControlPanel.window, "", "", FILE_TYPES)[0]
        print(filePath)
        self.loadJson(filePath)
        print("")
    def loadJson(self, filePath):
        try:
            objects = load_frontend_from_json(filePath)
            ControlPanel.features[Mode.START.value].start = objects[0]
            ControlPanel.features[Mode.FINISH.value].finish = objects[1]
            size = objects[2]
            boxes = objects[3]
            ControlPanel.features[Mode.WALLS.value].walls = objects[4]
            ControlPanel.features[Mode.SIGNS.value].Signs = objects[5]
            ControlPanel.PrintObjects()
            ControlPanel.window.update()
        except:
            print("Error: is file {0} exist?".format(filePath))


class GenerateJson(BaseGuiObject):
    def processButtonPressing(self):
        print("\nGenerate json:")
        start = ControlPanel.features[Mode.START.value].start
        finish = ControlPanel.features[Mode.FINISH.value].finish
        self.__walls = ControlPanel.features[Mode.WALLS.value].walls
        self.__signs = ControlPanel.features[Mode.SIGNS.value].Signs
        if start is not None:
            create_json_from_gui(start, finish, Map.CELLS_AMOUNT, Map.CELLS_SIZE, 
                                 Map.SIZE, None, self.__walls, self.__signs)
            ControlPanel.PrintObjects()
        else:
            print("Warning: firstly, you should set start position.")
        print("")


class CreateSdf(BaseGuiObject):
    def processButtonPressing(self):
        print("\nCreate sdf:")
        create_sdf_from_json()
        print("")
