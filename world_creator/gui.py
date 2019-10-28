#!/usr/bin/env python3
import sys, random
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, QLabel, \
    QDialog, QFileDialog, QLineEdit, QVBoxLayout
from PyQt5.QtGui import QPainter, QColor, QFont, QPen, QBrush, QIcon, QPixmap, QImage
from PyQt5.QtCore import Qt, QSize
from enum import Enum
from json_converter import *
from json_constants import *

# Constants:
class Mode(Enum):
    NO_MODE = int(-1)
    CHOOSE_MAP_SIZE = int(0)
    CHOOSE_CELL_SIZE = int(1)
    CHOOSE_START_POSITION = int(2)
    CHOOSE_END_POSITION = int(3)
    CREATE_BOXES = int(4)
    CREATE_WALLS = int(5)
    DELETE_WALLS = int(6)
    CREATE_SIGNS = int(7)
    CREATE_LIGHTS = int(8)

class CollorCode(Enum):
    WHITE = str("FFFFFF")
    BLUE = str("0000FF")
    GREEN = str("00FF00")
    RED = str("FF0000")

# Global objects
Walls = list()
Signs = list()

class SignChoiceDialog(QDialog):
    def __init__(self, mainWindow, pos, parent=None):
        super(SignChoiceDialog, self).__init__(parent)
        self.pos = pos
        self.mainWindow = mainWindow
        self.createDialog()
    def createDialog(self):
        self.dialog = QDialog()
        self.dialog.label = QLabel("Choose the sign:")
        self.dialog.buttons = list()
        self.dialog.buttons.append(QPushButton("0. Empty"))
        self.dialog.buttons.append(QPushButton("1. Stop sign"))
        self.dialog.buttons.append(QPushButton("2. Only forward sign"))
        self.dialog.buttons.append(QPushButton("3. Only left sign"))
        self.dialog.buttons.append(QPushButton("4. Only right sign"))
        self.dialog.buttons.append(QPushButton("5. Only forward or left sign"))
        self.dialog.buttons.append(QPushButton("6. Only forward or right sign"))

        self.dialog.buttons[0].clicked.connect(self.setEmpty)
        self.dialog.buttons[1].clicked.connect(self.setStopSign)
        self.dialog.buttons[1].setIcon(QIcon(ImagesPaths.STOP))
        self.dialog.buttons[2].clicked.connect(self.setOnlyForwardSign)
        self.dialog.buttons[2].setIcon(QIcon(ImagesPaths.ONLY_FORWARD))
        self.dialog.buttons[3].clicked.connect(self.setOnlyLeftSign)
        self.dialog.buttons[3].setIcon(QIcon(ImagesPaths.ONLY_LEFT))
        self.dialog.buttons[4].clicked.connect(self.setOnlyRightSign)
        self.dialog.buttons[4].setIcon(QIcon(ImagesPaths.ONLY_RIGHT))
        self.dialog.buttons[5].clicked.connect(self.setForwardOrLeftSign)
        self.dialog.buttons[5].setIcon(QIcon(ImagesPaths.FORWARD_OR_LEFT))
        self.dialog.buttons[6].clicked.connect(self.setForwardOrRightSign)
        self.dialog.buttons[6].setIcon(QIcon(ImagesPaths.FORWARD_OR_RIGHT))
        layout = QGridLayout()
        layout.addWidget(self.dialog.label, 0, 0)
        for i in range(0, len(self.dialog.buttons)):
            layout.addWidget(self.dialog.buttons[i], i, 0)
            self.dialog.buttons[i].setIconSize(QSize(24, 24))
        self.dialog.setLayout(layout)
        self.dialog.show()
    def setEmpty(self):
        self.deleteSign(self.pos)
    def setStopSign(self):
        self.addSign(self.pos, ImagesPaths.STOP)
    def setOnlyForwardSign(self):
        self.addSign(self.pos, ImagesPaths.ONLY_FORWARD)
    def setOnlyLeftSign(self):
        self.addSign(self.pos, ImagesPaths.ONLY_LEFT)
    def setOnlyRightSign(self):
        self.addSign(self.pos, ImagesPaths.ONLY_RIGHT)
    def setForwardOrLeftSign(self):
        self.addSign(self.pos, ImagesPaths.FORWARD_OR_LEFT)
    def setForwardOrRightSign(self):
        self.addSign(self.pos, ImagesPaths.FORWARD_OR_RIGHT)

    def addSign(self, poseIndexes, signImg):
        print("Sign {1} was added in {0}.".format(poseIndexes, signImg))
        Signs.append([poseIndexes, signImg])
        self.dialog.close()
        self.mainWindow.update()
    def deleteSign(self, poseIndexes):
        for sign in Signs:
            if sign[0] == poseIndexes:
                print("Sign was deleted: " + str(poseIndexes))
                Signs.remove(sign)
        self.dialog.close()
        self.mainWindow.update()


class AbstactFeature():
    def __init__(self, button):
        self.button = button
        self.button.setEnabled(False)
    def processButtonPressing(self):
        pass
    def processMousePressing(self):
        pass
    def draw(self):
        pass


class StartPosition(AbstactFeature):
    def __init__(self, button):
        self.button = button
        button.pressed.connect(self.processButtonPressing)
        self.start = None
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.CHOOSE_START_POSITION)
    def processMousePressing(self, e):
        self.start = MainWindow.calculateCellIndexes(e.pos().x(), e.pos().y())
        print("start pose was setted using mouse: " + str(self.start))


class Wall(AbstactFeature):
    def __init__(self, button):
        self.button = button
        button.pressed.connect(self.processButtonPressing)
        self.lastClickNumber = 0
        self.pressedFirstNode = None
        self.pressedSecondNode = None
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.CREATE_WALLS)
    def processMousePressing(self, e):
        pos = MainWindow.calculateNodeIndexes(e.pos().x(), e.pos().y())
        if self.lastClickNumber is 1:
            self.lastClickNumber = 2
            self.pressedSecondNode = pos
            self.addWall([self.pressedFirstNode, self.pressedSecondNode])
        else:
            self.lastClickNumber = 1
            self.pressedFirstNode = pos
    def addWall(self, nodesIndexes):
        """
        @brief Try to add new wall
        """
        if self.isWallPossible(nodesIndexes) is True:
            print("Wall was added: " + str(nodesIndexes))
            Walls.append(nodesIndexes)
    def isWallPossible(self, nodesIndexes):
        """
        @brief Check if the wall is possible
        @note print the reason if wall is not possible
        """
        if self.isWallOutOfRange(nodesIndexes) is True:
            print("Warning: wall out of map: " + str(nodesIndexes))
        elif self.isThisWallPoint(nodesIndexes) is True:
            print("Warning: wall can't be point: " + str(nodesIndexes))
        elif self.isThisWallDiagonal(nodesIndexes) is True:
            print("Warning: wall can't be diagonal: " + str(nodesIndexes))
        elif self.isThereConflictBetweenWalls(nodesIndexes) is True:
            print("Warning: there is conflict between existing walls \
            and this: " + str(nodesIndexes))
        else:
            return True
        return False
    def isThisWallPoint(self, nodesIndexes):
        return nodesIndexes[0] == nodesIndexes[1]
    def isWallOutOfRange(self, nodesIndexes):
        return  nodesIndexes[0][0] > Map.CELLS_AMOUNT[0] or \
                nodesIndexes[0][0] < 0 or \
                nodesIndexes[1][0] > Map.CELLS_AMOUNT[0] or \
                nodesIndexes[1][0] < 0 or \
                nodesIndexes[0][1] > Map.CELLS_AMOUNT[1] or \
                nodesIndexes[0][1] < 0 or \
                nodesIndexes[1][1] > Map.CELLS_AMOUNT[1] or \
                nodesIndexes[1][1] < 0
    def isThisWallDiagonal(self, nodesIndexes):
        return ((self.isThisWallVertical(nodesIndexes) is False) and \
                (self.isThisWallHorizontal(nodesIndexes) is False))
    def isThisWallVertical(self, nodesIndexes):
        return nodesIndexes[0][0] == nodesIndexes[1][0]
    def isThisWallHorizontal(self, nodesIndexes):
        return nodesIndexes[0][1] == nodesIndexes[1][1]
    def isThereConflictBetweenWalls(self, nodesIndexes):
        return False


class DeleteWall(AbstactFeature):
    def __init__(self, button):
        self.button = button
        button.pressed.connect(self.processButtonPressing)
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.DELETE_WALLS)
    def processMousePressing(self, e):
        pos = MainWindow.calculateEdgeIndexes(e.pos().x(), e.pos().y())
        self.deleteWall(pos)
    def deleteWall(self, pos):
        """
        @brief Delete wall
        """
        print(Walls)
        try:
            isVerticalFound = False
            isHorizontalFound = False
            for wall in Walls:
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
                Walls.remove(wall)
            elif isHorizontalFound == True:
                print("delete horizontal: " + str(wall) + " and " + str(pos))
                Walls.remove(wall)
            ControlPanel.window.update()
        except:
            print("Warning: it's not wall")


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

        features = list()

        ControlPanel.buttons = list()

        ControlPanel.buttons.append(ControlPanel.createButton('1. Choose map size'))
        features.append(AbstactFeature(ControlPanel.buttons[0]))
        window.layout.addWidget(ControlPanel.buttons[0], 1, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('2. Choose cells size'))
        features.append(AbstactFeature(ControlPanel.buttons[1]))
        window.layout.addWidget(ControlPanel.buttons[1], 2, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('3. Choose start pose'))
        features.append(StartPosition(ControlPanel.buttons[2]))
        window.layout.addWidget(ControlPanel.buttons[2], 3, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('4. Choose end pose'))
        features.append(AbstactFeature(ControlPanel.buttons[3]))
        window.layout.addWidget(ControlPanel.buttons[3], 4, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('5. Create boxes'))
        features.append(AbstactFeature(ControlPanel.buttons[4]))
        window.layout.addWidget(ControlPanel.buttons[4], 5, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('6. Create walls'))
        features.append(Wall(ControlPanel.buttons[5]))
        window.layout.addWidget(ControlPanel.buttons[5], 6, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('7. Delete walls'))
        features.append(DeleteWall(ControlPanel.buttons[6]))
        window.layout.addWidget(ControlPanel.buttons[6], 7, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('8. Create signs'))
        ControlPanel.buttons[7].pressed.connect(window.createSignsCallback)
        window.layout.addWidget(ControlPanel.buttons[7], 8, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('9. Create lights'))
        ControlPanel.buttons[8].pressed.connect(window.createLightsCallback)
        ControlPanel.buttons[8].setEnabled(False)
        window.layout.addWidget(ControlPanel.buttons[8], 9, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('Load json'))
        ControlPanel.buttons[9].pressed.connect(window.loadJsonCallback)
        window.layout.addWidget(ControlPanel.buttons[9], 11, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('Generate json'))
        ControlPanel.buttons[10].pressed.connect(window.generateJsonCallback)
        window.layout.addWidget(ControlPanel.buttons[10], 13, 1)

        ControlPanel.buttons.append(ControlPanel.createButton('Create sdf world from json'))
        ControlPanel.buttons[11].pressed.connect(window.createSdfCallback)
        window.layout.addWidget(ControlPanel.buttons[11], 14, 1)

        window.layout.addWidget(QLabel('To create the world:', window), 0, 1)
        window.layout.addWidget(QLabel('Or use these features:', window), 10, 1)
        window.layout.addWidget(QLabel('Then press buttons below:',window), 12, 1)

        return features

    @staticmethod
    def SetMode(mode):
        """
        @brief set mode from class Mode(enum)
        """
        try:
            for i in range(0, 10):
                ControlPanel.setButtonCollor(ControlPanel.buttons[i], CollorCode.WHITE)
            ControlPanel.setButtonCollor(ControlPanel.buttons[mode.value], CollorCode.RED)
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

class MainWindow(QWidget):
# *************** High level methods which allow create window ***************
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.initUI()
        Map.Init([18, 18])
        self.initFeatures()
        features = ControlPanel.Init(self)
        self.startPosition = features[2]
        self.wall = features[5]
        self.deleteWall = features[6]

    def initUI(self):
        self.setGeometry(300, 300, 710, 320)
        self.setWindowTitle('World creator v.2')
        self.show()

    def initFeatures(self):
        self.end = None

# ************** Control methods which allow to choose mode ******************
    def createSignsCallback(self):
        ControlPanel.SetMode(Mode.CREATE_SIGNS)
    def createLightsCallback(self):
        pass
    def loadJsonCallback(self):
        global Walls, Signs
        FILE_TYPES = "Json Files (*.json)"
        filePath = QFileDialog.getOpenFileName(self, "", "", FILE_TYPES)[0]
        objects = load_frontend_from_json(filePath)
        self.startPosition.start = objects[0]
        finish = objects[1]
        size = objects[2]
        boxes = objects[3]
        Walls = objects[4]
        Signs = objects[5]
        print("Loaded from json:")
        print("start: ", self.startPosition.start)
        print("finish: ", finish)
        print("size: ", size)
        print("boxes: ", boxes)
        print("Walls: ", Walls)
        print("Signs: ", Signs)
        self.update()
    def generateJsonCallback(self):
        if self.startPosition.start is not None:
            create_json_from_gui(self.startPosition.start, Map.SIZE, None, Walls, Signs)
            print("generate json:")
            print("start: ", self.startPosition.start)
            print("finish: ", " ")
            print("size: ", Map.SIZE)
            print("boxes: ", " ")
            print("Walls: ", Walls)
            print("Signs: ", Signs)
        else:
            print("Warning: firstly, you should set start position.")
    def createSdfCallback(self):
        create_sdf_from_json()

# ************ Methods which allow to create and delete walls ****************
    def mousePressEvent(self, e):
        pos = e.pos()
        if ControlPanel.mode is Mode.CHOOSE_START_POSITION:
            self.startPosition.processMousePressing(e)
        elif ControlPanel.mode is Mode.CREATE_WALLS:
            self.wall.processMousePressing(e)
        elif ControlPanel.mode is Mode.DELETE_WALLS:
            self.deleteWall.processMousePressing(e)
        elif ControlPanel.mode is Mode.CREATE_SIGNS:
            pos = self.calculatePositionIndexes(pos.x(), pos.y())
            self.signChoiceDialog = SignChoiceDialog(self, pos)
        else:
            print("Warning: you should choose mode")
        self.update()

    def addBox(self, pos):
        pass


    def paintEvent(self, event=None):
        qp = QPainter()
        qp.begin(self)
        self.drawPoints(qp)
        for wall in Walls:
            self.drawWall(qp, wall[0], wall[1])
        self.drawTable(qp)
        if self.startPosition.start is not None:
            self.drawBox(qp, self.startPosition.start)
        for sign in Signs:
            self.drawSign(qp, sign[0], sign[1])

        qp.end()


    def drawBox(self, qp, cellIndexes):
        """
        @brief Draw box on table
        @param cellIndexes - x and y indexes from 0 to Map.CELLS_AMOUNT - 1
        """
        centerPos = self.calculateRealPositionByCellIndexes(cellIndexes)
        self.drawRectangle(qp, centerPos, ControlPanel.CellsSize)


    def drawWall(self, qp, indexesOfNode1, indexesOfNode2):
        """
        @brief Draw wall on table
        @param indexesOfNode1 - x and y indexes from 0 to Map.CELLS_AMOUNT - 1
        @param indexesOfNode2 - x and y indexes from 0 to Map.CELLS_AMOUNT - 1
        """
        posOfNode1 = self.calculateRealPositionByNodeIndexes(indexesOfNode1)
        posOfNode2 = self.calculateRealPositionByNodeIndexes(indexesOfNode2)
        self.drawLine(qp, posOfNode1, posOfNode2)


    def drawSign(self, qp, poseIndexes, imgPath):
        """
        @brief Draw sign on table
        @param indexesOfNode1 - x and y indexes from 0 to Map.CELLS_AMOUNT - 1
        """
        centerPos = self.calculateRealPositionByPoseIndexes(poseIndexes)
        self.drawImg(qp, centerPos, imgPath)

# *************** Low level methods: raw draw and calculations ***************
# *************** Basicaly methods below work with real window positions *****
    @staticmethod
    def calculateCellIndexes(point_x, point_y):
        """
        @brief Calculate cell coordinate using real mouse position on window
        """
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        node = [int()] * 2

        for axe in range(0, 2):
            node[axe] = int(tablePose[axe] / ControlPanel.CellsSize[axe])
            if node[axe] > (Map.CELLS_AMOUNT[axe] + 1) or (node[axe] < 0):
                return None
        return node

    @staticmethod
    def calculateNodeIndexes(point_x, point_y):
        """
        @brief Calculate node coordinate using real mouse position on window
        @note node indexes will be out of rate if point coordinate out of rate
        """
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
    def calculateEdgeIndexes(point_x, point_y):
        """
        @brief Calculate edge coordinate using real mouse position on window
        @note edge indexes will be out of rate if point coordinate out of rate
        """
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
      

    def calculatePositionIndexes(self, point_x, point_y):
        """
        @brief Calculate position coordinate using real mouse position on window
        @note pose indexes will be out of rate if point coordinate out of rate
        """
        tablePose = [point_x - ControlPanel.Left, point_y - ControlPanel.Top]
        pose = [int()] * 2

        for axe in range(0, 2):
            meterSize = ControlPanel.CellsSize[axe] / Map.CELLS_SIZE_IN_METERS[axe]
            pose[axe] = int(tablePose[axe] / meterSize)
            # Line below is needed because of unexpected work of division of
            # negative nubmers  
            if tablePose[axe] < 0: pose[axe] -= 1 
        return pose


    def calculateRealPositionByCellIndexes(self, cellIndexes):
        """
        @brief Calculate real node coordinate using node indexes
        """
        return [ControlPanel.Left + (cellIndexes[0] + 1) * ControlPanel.CellsSize[0],
                ControlPanel.Top + (cellIndexes[1] + 1) * ControlPanel.CellsSize[1]]
      
    def calculateRealPositionByPoseIndexes(self, poseIndexes):
        """
        @brief Calculate real node coordinate using node indexes
        """
        return [ControlPanel.Left + (poseIndexes[0] + 1) * ControlPanel.CellsSize[0]/2,
                ControlPanel.Top + (poseIndexes[1] + 1) * ControlPanel.CellsSize[1]/2]


    def calculateRealPositionByNodeIndexes(self, nodeIndexes):
        """
        @brief Calculate real node coordinate using node indexes
        """
        return [ ControlPanel.Left + nodeIndexes[0] * ControlPanel.CellsSize[0],
                 ControlPanel.Top + nodeIndexes[1] * ControlPanel.CellsSize[1] ]

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


    def drawLine(self, qp, firstPoseOnWindow, secondPoseOnWindow):
        """ 
        @brief Draw line
        @note it uses real window coordinates 
        """
        pen = QPen(Qt.black, 3, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawLine(*firstPoseOnWindow, *secondPoseOnWindow)


    def drawRectangle(self, qp, centerPosition, size, collor=QColor(255, 0, 0)):
        """ 
        @brief Draw rectangle
        @note it uses real window coordinates 
        """
        brush = QBrush(collor)
        qp.setBrush(brush)
        left = centerPosition[0] - ControlPanel.CellsSize[0]
        top = centerPosition[1] - ControlPanel.CellsSize[1]
        qp.drawRect(left, top, ControlPanel.CellsSize[0], ControlPanel.CellsSize[1])


    def drawImg(self, qp, centerPosition, imgPath = ImagesPaths.STOP):
        """ 
        @brief Draw a sign
        @note it uses real window coordinates 
        """
        halfCellsSizes = [ControlPanel.CellsSize[0]/2, ControlPanel.CellsSize[1]/2]
        left = centerPosition[0] - halfCellsSizes[0]
        top = centerPosition[1] - halfCellsSizes[1]
        img = QImage(imgPath).scaled(QSize(halfCellsSizes[0], halfCellsSizes[1]))
        qp.drawImage(left, top, img)


    def drawTable(self, qp):
        """ 
        @brief Draw table with cells
        """
        thinPen = QPen(Qt.black, 0.5, Qt.SolidLine)
        qp.setPen(thinPen)

        windowWidth = self.frameGeometry().width()
        windowHeight = self.frameGeometry().height()

        ControlPanel.Right = int(0.70 * windowWidth)
        ControlPanel.Left = int(0.05 * windowWidth)
        ControlPanel.Bot = int(0.9 * windowHeight)
        ControlPanel.Top = int(0.05 * windowHeight)

        # It is important that table sizes must divided on cell size (or 
        # amount) without remainder
        ControlPanel.Left = ControlPanel.Left - ((ControlPanel.Left - ControlPanel.Right) % Map.CELLS_AMOUNT[0])
        ControlPanel.Top = ControlPanel.Top - ((ControlPanel.Top - ControlPanel.Bot) % Map.CELLS_AMOUNT[1])

        self.tableWidth = ControlPanel.Right - ControlPanel.Left
        self.tableHeight = ControlPanel.Bot - ControlPanel.Top
        ControlPanel.CellsSize = [ int(self.tableWidth/Map.CELLS_AMOUNT[0]), 
                                   int(self.tableHeight/Map.CELLS_AMOUNT[1]) ]

        for row in range(ControlPanel.Top, ControlPanel.Bot + 1, ControlPanel.CellsSize[1]):
            qp.drawLine(ControlPanel.Left, row, ControlPanel.Right, row)
        for col in range(ControlPanel.Left, ControlPanel.Right + 1, ControlPanel.CellsSize[0]):
            qp.drawLine(col, ControlPanel.Top, col, ControlPanel.Bot)


