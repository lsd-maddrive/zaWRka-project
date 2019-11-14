#!/usr/bin/env python3
import sys, random
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, \
    QLabel, QDialog, QStatusBar, QMainWindow
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
    GENERATE_JSON = int(9)
    GENERATE_SDF = int(10)

class ColorCode(Enum):
    WHITE = str("FFFFFF")
    BLUE = str("0000FF")
    GREEN = str("00FF00")
    RED = str("FF0000")

class ObjectType(Enum):
    START = 10,
    WALL = 11,
    BOX = 12,
    SQUARE = 13,
    SIGN = 14,
    TRAFFIC_LIGHT = 15
    
# ***************************** Main window *********************************
class Canvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
    def paintEvent(self, event=None):
        self.cellSz = Size2D(int(self.geometry().width() // Map.CELLS_AMOUNT.x), int(self.geometry().height() // Map.CELLS_AMOUNT.y))
        self.canvasSz = Size2D(self.cellSz.x * Map.CELLS_AMOUNT.x, self.cellSz.y * Map.CELLS_AMOUNT.y)
        
        qp = MyPainter(self, self.cellSz)

        self.drawMap(qp)
    
        for obj_type, objs in ControlPanel.objects.items():
            if type(objs) is list:
                [obj.render(qp) for obj in objs] 
            else:
                objs.render(qp)
                
    
    @staticmethod
    def getCellClicked(pos, cell_sz):
        return Point2D(pos.x // cell_sz.x, pos.y // cell_sz.y)
        
    @staticmethod
    def getCrossClicked(pos, cell_sz):
        # Cross == Node (crossing of lines)
        return Point2D((pos.x + cell_sz.x/2.) // cell_sz.x, (pos.y + cell_sz.y/2.) //cell_sz.y)
    
    # Not really good to use Sign class info
    @staticmethod
    def getCellSignQuarter(pos, cell_sz):
        cell = Canvas.getCellClicked(pos, cell_sz)
        float_cell = Point2D(pos.x / float(cell_sz.x), pos.y / float(cell_sz.y))

        orient = None
        if float_cell.x - cell.x > 0.5:
            if float_cell.y - cell.y > 0.5:
                orient = CellQuarter.RIGHT_BOT
            else:
                orient = CellQuarter.RIGHT_TOP
        else:
            if float_cell.y - cell.y > 0.5:
                orient = CellQuarter.LEFT_BOT
            else:
                orient = CellQuarter.LEFT_TOP
        
        return (cell, orient)
        
    def get_canvas_pos(self, x, y):
        if x >= self.canvasSz.x:
            x = self.canvasSz.x-1
        
        if y >= self.canvasSz.y:
            y = self.canvasSz.y-1
        
        return Point2D(x, y)
        
    def mousePressEvent(self, e):
        canvasPos = self.get_canvas_pos(e.pos().x(), e.pos().y()) 
        
        if ControlPanel.mode is not Mode.NO_MODE:
            if e.button() == Qt.LeftButton:
                ControlPanel.modes[ControlPanel.mode].processLeftMousePressing(canvasPos, self.cellSz)
            elif e.button() == Qt.RightButton:
                ControlPanel.modes[ControlPanel.mode].processRightMousePressing(canvasPos, self.cellSz)
        else:
            print("Warning: you should choose mode")
        self.update()
        
             
    def drawMap(self, qp):
        """ 
        @brief Draw map with cells
        """
        thinPen = QPen(Qt.black, 1, Qt.SolidLine)
        qp.setPen(thinPen)
        qp.drawRect(0, 0, self.canvasSz.x, self.canvasSz.y)
        for row in range(1, Map.CELLS_AMOUNT.y):
            qp.drawLine(0, row*self.cellSz.y, self.canvasSz.x, row*self.cellSz.y)
        for col in range(1, Map.CELLS_AMOUNT.x):
            qp.drawLine(col*self.cellSz.x, 0, col*self.cellSz.x, self.canvasSz.y)
    

class MainWindow(QWidget):
    def __init__(self, filePath, saveFilePrefix):
        super().__init__()
        WINDOW_TITLE = 'World creator v.2'
        self.setWindowTitle(WINDOW_TITLE)
        self.show()
        
        ControlPanel.Init(self, saveFilePrefix)

    def paintEvent(self, event=None):
        qp = QPainter(self)
        self.drawPoints(qp)

    def drawPoints(self, qp):
        """ 
        @brief Draw window background with random points cloud
        """
        qp.setPen(Qt.red)
        windowSize = self.size()
        for i in range(1000):
            x = random.randint(1, windowSize.width() - 1)
            y = random.randint(1, windowSize.height() - 1)
            qp.drawPoint(x, y)


# ********************************* Map *************************************
class Map:
    """
    Static class with map settings
    """
    @staticmethod
    def Init(cellsAmount, cellsSize):
        Map.CELLS_AMOUNT = cellsAmount
        Map.CELLS_SIZE = cellsSize
        Map.SIZE = Size2D(cellsAmount.x * cellsSize.x, 
                          cellsAmount.y * cellsSize.y)
        print("World cells: amount={0},size={1}".format(cellsAmount,cellsSize))


def generateOutputFiles(self):
    print("\nGenerate json/world:")
    start = ControlPanel.features[Mode.START.value].start
    finish = ControlPanel.features[Mode.FINISH.value].finish
    self.__walls = ControlPanel.features[Mode.WALLS.value].walls
    self.__signs = ControlPanel.features[Mode.SIGNS.value].Signs
    if start is not None:
        create_json_from_gui(start,finish,Map.CELLS_AMOUNT,Map.CELLS_SIZE,
            Map.SIZE, None, self.__walls, self.__signs, ControlPanel.jsonSaveFpath)
        create_sdf_from_gui(start,finish,Map.CELLS_AMOUNT,Map.CELLS_SIZE, 
                                Map.SIZE, None, self.__walls, self.__signs, ControlPanel.worldSaveFpath)
        ControlPanel.PrintObjects()
    else:
        print("Warning: firstly, you should set start position.")
    print("")


# ***************************** Control panel ********************************
class ControlPanel():
    @staticmethod
    def Init(window, saveFilePrefix):
        ControlPanel.mode = Mode.NO_MODE
        ControlPanel.window = window

        ControlPanel.jsonSaveFpath = saveFilePrefix + '.json'
        ControlPanel.worldSaveFpath = saveFilePrefix + '.world'

        ControlPanel.canvas = Canvas(window)

        window.layout = QGridLayout()
        window.setLayout(window.layout)
        
        window.layout.setSpacing(5)

        ControlPanel.modes = {
            Mode.START: GuiStart('1. Choose start pose'),
            # Mode.BOXES: BaseGuiObject('2. Create boxes', False),
            Mode.WALLS: GuiWallsMode('2. Create walls'),
            Mode.SIGNS: GuiSigns('3. Create signs'),
        }
        
        ControlPanel.objects = {
            ObjectType.TRAFFIC_LIGHT: [],
            ObjectType.WALL: [],
            ObjectType.SIGN: []
        }
    
        ControlPanel.statusBar = QStatusBar()
        ControlPanel.statusBar.showMessage('Ready')
        ControlPanel.statusBar.setMaximumHeight(20)
    
        generateButton = ControlPanel.createButton('Generate JSON/WORLD')
        generateButton.pressed.connect(generateOutputFiles)
        
        # Layout fill
        window.layout.addWidget(QLabel('To create the world:', window), 0, 1)
        for idx, mode in enumerate(ControlPanel.modes):
            window.layout.addWidget(ControlPanel.modes[mode].button, idx + 1, 1)        
        window.layout.addWidget(QLabel('Then press buttons below:',window), len(ControlPanel.modes)+1, 1)
        window.layout.addWidget(generateButton, len(ControlPanel.modes)+2, 1)
        
        window.layout.addWidget(ControlPanel.statusBar, len(ControlPanel.modes)+3, 0, 1, 2)

        window.layout.addWidget(ControlPanel.canvas, 0, 0, len(ControlPanel.modes)+3, 1)
    
        
    @staticmethod
    def show_message(msg):
        ControlPanel.statusBar.showMessage(msg)
    
    @staticmethod
    def SetMode(set_mode):
        """
        @brief set mode from class Mode(enum)
        """
        print(set_mode)
        for mode in ControlPanel.modes:
            ControlPanel.setButtonColor(ControlPanel.modes[mode].button, ColorCode.WHITE)
            
        if set_mode != Mode.NO_MODE:
            ControlPanel.setButtonColor(ControlPanel.modes[set_mode].button, ColorCode.RED)
        
        ControlPanel.mode = set_mode

    @staticmethod
    def createButton(name):
        but = QPushButton(name, ControlPanel.window)
        but.setFixedSize(QSize(200, 25))
        return but

    @staticmethod
    def setButtonColor(btn, color = ColorCode.WHITE):
        btn.setStyleSheet("background-color: #{}".format(color.value))
        
    @staticmethod
    def PrintObjects():
        print("Gui objects are:")
        print("- start:", ControlPanel.features[Mode.START.value].start)
        print("- finish:", ControlPanel.features[Mode.FINISH.value].finish)
        print("- cellsAmount:", Map.CELLS_AMOUNT)
        print("- cellsSize:", Map.CELLS_SIZE)
        print("- mapSize:", Map.SIZE)
        print("- boxes:", " ")
        print("- walls:", ControlPanel.features[Mode.WALLS.value].walls)
        print("- signs:", ControlPanel.features[Mode.SIGNS.value].Signs)


# ***************************** BaseGuiObject ********************************
class BaseGuiObject():
    """ @brief Interface for button features """
    def __init__(self, buttonText, isItEnable=True):
        """ @brief Init button, you shouldn't override this constuctor """
        self.button = ControlPanel.createButton(buttonText)
        self.button.pressed.connect(self.processButtonPressing)
        self.button.setEnabled(isItEnable)
    def processButtonPressing(self):
        pass
    def processRightMousePressing(self, canvas_pos, canvas_cell_sz):
        pass
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        pass


class GuiStart(BaseGuiObject):
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.START)
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        clickCell = Canvas.getCellClicked(canvas_pos, canvas_cell_sz)
        ControlPanel.objects[ObjectType.START] = Start(clickCell)
        print("start pose was setted using mouse:", ControlPanel.objects[ObjectType.START])
    

class GuiWallsMode(BaseGuiObject):
    def __init__(self, buttonText, isItEnable=True):
        super().__init__(buttonText, isItEnable)
        self._prev_clicked_cross = None
        
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.WALLS)
        self._prev_clicked_cross = None
        ControlPanel.show_message('Select first point on canvas')
        
    def processRightMousePressing(self, canvas_pos, canvas_cell_sz):
        #J ust repeat reset logic
        self.processButtonPressing()
    
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        clickCross = Canvas.getCrossClicked(canvas_pos, canvas_cell_sz)
        
        if self._prev_clicked_cross is not None:
            ControlPanel.objects[ObjectType.WALL] += [Wall(clickCross, self._prev_clicked_cross)]
        
        self._prev_clicked_cross = clickCross
        

class GuiSigns(BaseGuiObject):
    def _initOtherParameters(self):
        self.Signs = Signs()
    def processButtonPressing(self):
        ControlPanel.SetMode(Mode.SIGNS)
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        click_cell, orient = Canvas.getCellSignQuarter(canvas_pos, canvas_cell_sz)
        self.signChoiceDialog = SignChoiceDialog(click_cell, orient)


class SignChoiceDialog(QDialog):
    def __init__(self, pos, orient, parent=None):
        super(SignChoiceDialog, self).__init__(parent)
        self.__pos = pos
        self.__orient = orient
        
        self.__window = ControlPanel.window
        self.__createDialog()
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
        callback = lambda: self.__deleteSign(self.__pos, self.__orient)
        self.__dialog.buttons[0].clicked.connect(callback)
        layout.addWidget(self.__dialog.buttons[0], 1, 0)
        
        for i in range(1, len(info)):
            self.__dialog.buttons.append(QPushButton(info[i][0]))
            self.__dialog.buttons[i].setIcon(QIcon(info[i][1]))
            self.__dialog.buttons[i].setIconSize(QSize(24, 24))
            callback = lambda this, row=i: self.__addSign(self.__pos, self.__orient, info[row][1])
            self.__dialog.buttons[i].clicked.connect(callback)
            layout.addWidget(self.__dialog.buttons[i], i+1, 0)
        
        self.__dialog.setLayout(layout)
        self.__dialog.show()
        
    def __addSign(self, pos, orient, signImg):
        for idx, sign in enumerate(ControlPanel.objects[ObjectType.SIGN]):
            if sign.point == pos and sign.orient == orient:
                print("Delete object: sign ({2}) with pose {0}/{1}".format(pos, orient.name, sign.type))
                ControlPanel.objects[ObjectType.SIGN].remove(sign)
                
        print("Add object: sign ({2}) with pose {0}/{1}.".format(pos, orient.name, signImg))
        ControlPanel.objects[ObjectType.SIGN] += [Sign(pos, orient, signImg)]
        self.__dialog.close()
        self.__window.update()
        

class LoadJson(BaseGuiObject):
    def processButtonPressing(self):
        print("\nLoad json:")
        FILE_TYPES = "Json Files (*.json)"
        filePath = QFileDialog.getOpenFileName(ControlPanel.window, "", " ",
                                               FILE_TYPES)[0]
        print(filePath)
        self.loadJson(filePath)
        print("")
    def loadJson(self, filePath):
        if filePath == "":
            print("User don't choose the file to save.")
        else:
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
                print("Error: file {0} is incorrect!".format(filePath))

