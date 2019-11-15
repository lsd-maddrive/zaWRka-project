#!/usr/bin/env python3
import sys, random
from PyQt5.QtWidgets import QWidget, QApplication, QPushButton, QGridLayout, \
    QLabel, QDialog, QStatusBar, QMainWindow, QButtonGroup
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QIcon, QImage
from PyQt5.QtCore import Qt, QSize
from enum import Enum
import converter
from objects import *
from json_constants import *
import logging as log

log.basicConfig(filename='world_creator.log', level=log.DEBUG)
log.getLogger().addHandler(log.StreamHandler(sys.stdout))

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

    
# ***************************** Main window *********************************
class Canvas(QWidget):
    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        
    def paintEvent(self, event=None):
        self.cellSz = Size2D(int(self.geometry().width() // self.model.map_params.n_cells.x), 
                             int(self.geometry().height() // self.model.map_params.n_cells.y))
        self.canvasSz = Size2D(self.cellSz.x * self.model.map_params.n_cells.x, 
                               self.cellSz.y * self.model.map_params.n_cells.y)
        
        qp = MyPainter(self, self.cellSz)

        self.drawMap(qp)
    
        for obj_type, objs in self.model.objects.items():
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
    
    @staticmethod
    def getCellQuarter(pos, cell_sz):
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
        
        if self.model.mode is not Mode.NO_MODE:
            if e.button() == Qt.LeftButton:
                self.model.modes[self.model.mode].processLeftMousePressing(canvasPos, self.cellSz)
            elif e.button() == Qt.RightButton:
                self.model.modes[self.model.mode].processRightMousePressing(canvasPos, self.cellSz)
        else:
            print("Warning: you should choose mode")
            
        print('Canvas update call')
        self.update()
        
             
    def drawMap(self, qp):
        thinPen = QPen(Qt.black, 1, Qt.SolidLine)
        qp.setPen(thinPen)
        qp.drawRect(0, 0, self.canvasSz.x, self.canvasSz.y)
        for row in range(1, self.model.map_params.n_cells.y):
            qp.drawLine(0, row*self.cellSz.y, self.canvasSz.x, row*self.cellSz.y)
        for col in range(1, self.model.map_params.n_cells.x):
            qp.drawLine(col*self.cellSz.x, 0, col*self.cellSz.x, self.canvasSz.y)
    
    
class Model:
    # Class for keeping main processing data
    def __init__(self, map_params, load_filepath):
        self.modes = {
            Mode.START: GuiStartMode(self),
            Mode.WALLS: GuiWallsMode(self),
            Mode.SIGNS: GuiSignsMode(self),
        }
        
        self.objects = {
            ObjectType.TRAFFIC_LIGHT: [],
            ObjectType.WALL: [],
            ObjectType.SIGN: [],
            ObjectType.SQUARE: []
        }
        
        if load_filepath:
            print('Loading data from {}'.format(load_filepath))
            self.map_params = converter.deserialize_from_json(load_filepath, self.objects)
        else:
            self.map_params = map_params
        
        self.mode = Mode.NO_MODE
    
    def set_mode(self, _set_mode):
        print(_set_mode)
        # TODO - View part / replace
        # for mode in self.modes:
            # self.set_button_color(self.modes[mode].button, ColorCode.WHITE)
            
        # if _set_mode != Mode.NO_MODE:
            # self.set_button_color(self.modes[_set_mode].button, ColorCode.RED)
        
        self.mode = _set_mode
        
    def set_button_color(self, btn, color = ColorCode.WHITE):
        btn.setStyleSheet("background-color: #{}".format(color.value))
        
    
class ModeButton(QPushButton):
    def __init__(self, text: str, mode: Mode, model: Model, parent=None):
        super().__init__(text, parent)
        self.setFixedSize(QSize(200, 25))
        self.setCheckable(True)
        
        self.mode = mode
        self.model = model

        self.clicked[bool].connect(self.clickHandler)

    def clickHandler(self, pressed):
        if pressed:
            self.model.set_mode(self.mode)
        else:
            self.model.set_mode(Mode.NO_MODE)
    
    
class MainWindow(QWidget):
    def __init__(self, load_filepath, save_file_prefix, map_params):
        super().__init__()
        WINDOW_TITLE = 'World creator v.2'
        self.setWindowTitle(WINDOW_TITLE)
        self.show()
        
        self.model = Model(map_params, load_filepath)

        self.json_save_fpath = save_file_prefix + '.json'
        self.world_save_fpath = save_file_prefix + '.world'

        layout = QGridLayout()
        layout.setSpacing(5)
        self.setLayout(layout)
        
        generateButton = QPushButton('Generate JSON/WORLD', self)
        generateButton.setFixedSize(QSize(200, 25))
        generateButton.pressed.connect(self.generateOutputFiles)
        
        # TODO - maybe must be not "model" but "controller" connected to buttons
        mode_buttons = [
            ModeButton('1. Choose start pose', Mode.START, self.model, self),
            ModeButton('2. Create walls', Mode.WALLS, self.model, self),
            ModeButton('3. Create signs', Mode.SIGNS, self.model, self)
        ]        
        
        # Layout fill
        layout.addWidget(QLabel('To create the world:', self), 0, 1)
        for idx in range(len(mode_buttons)):
            layout.addWidget(mode_buttons[idx], idx + 1, 1)       

        layout.addWidget(QLabel('Then press buttons below:', self), len(mode_buttons)+1, 1)
        layout.addWidget(generateButton, len(mode_buttons)+2, 1)
        
        self.statusBar = QStatusBar()
        self.statusBar.showMessage('Ready')
        self.statusBar.setMaximumHeight(20)
        layout.addWidget(self.statusBar, len(mode_buttons)+3, 0, 1, 2)
        
        self.canvas = Canvas(self.model, self)
        layout.addWidget(self.canvas, 0, 0, len(mode_buttons)+3, 1)

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


    def generateOutputFiles(self):
        converter.serialize_2_json(self.json_save_fpath, self.model.objects, self.model.map_params)
        converter.create_sdf(self.world_save_fpath, self.model.objects, self.model.map_params)
        
        log.info("Files generated!")


# ***************************** BaseGuiMode ********************************
class BaseGuiMode():
    """ @brief Interface for button features """
    def __init__(self, model):
        self.model = model

    def processRightMousePressing(self, canvas_pos, canvas_cell_sz):
        pass
    
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        pass


class GuiStartMode(BaseGuiMode):
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        clickCell = Canvas.getCellClicked(canvas_pos, canvas_cell_sz)
        self.model.objects[ObjectType.START] = Start(clickCell)
        print("start pose was setted using mouse:", self.model.objects[ObjectType.START])
    

class GuiWallsMode(BaseGuiMode):
    def __init__(self, model):
        super().__init__(model)
        self._prev_clicked_cross = None

    def processRightMousePressing(self, canvas_pos, canvas_cell_sz):
        #Just repeat reset logic
        self._prev_clicked_cross = None
    
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        clickCross = Canvas.getCrossClicked(canvas_pos, canvas_cell_sz)
        
        if self._prev_clicked_cross is not None:
            self.model.objects[ObjectType.WALL] += [Wall(clickCross, self._prev_clicked_cross)]
        
        self._prev_clicked_cross = clickCross
        

class GuiSignsMode(BaseGuiMode):
    def processLeftMousePressing(self, canvas_pos, canvas_cell_sz):
        click_cell, orient = Canvas.getCellQuarter(canvas_pos, canvas_cell_sz)

        info = list([ ["0. Empty", None],
                      ["1. Stop", ImagesPaths.STOP],
                      ["2. Forward", ImagesPaths.ONLY_FORWARD],
                      ["3. Left", ImagesPaths.ONLY_LEFT],
                      ["4. Right", ImagesPaths.ONLY_RIGHT],
                      ["5. Forward or left", ImagesPaths.FORWARD_OR_LEFT],
                      ["6. Forward or right", ImagesPaths.FORWARD_OR_RIGHT],])

        self.signChoiceDialog = SignChoiceDialog(info)
        self.signChoiceDialog.exec_()
        
        select_idx = self.signChoiceDialog.get_result()
        if select_idx == 0:
            self.deleteSign(click_cell, orient)
        elif select_idx > 0:
            self.addSign(click_cell, orient, info[select_idx][1])

    def addSign(self, pos, orient, signImg):
        self.deleteSign(pos, orient)
        
        for idx, sign in enumerate(self.model.objects[ObjectType.SIGN]):
            if sign.point == pos and sign.orient == orient:
                print("Delete object: sign ({2}) with pose {0}/{1}".format(pos, orient.name, sign.type))
                self.model.objects[ObjectType.SIGN].remove(sign)
        
        sign_type = sign_path_to_sign_type(signImg)
        
        print("Add object: sign ({2}) with pose {0}/{1}.".format(pos, orient.name, sign_type))
        self.model.objects[ObjectType.SIGN] += [Sign(pos, orient, sign_type)]
    
    def deleteSign(self, pos, orient):
        for idx, sign in enumerate(self.model.objects[ObjectType.SIGN]):
            if sign.point == pos and sign.orient == orient:
                print("Delete object: sign ({2}) with pose {0}/{1}".format(pos, orient.name, sign.type))
                self.model.objects[ObjectType.SIGN].remove(sign)


class SignSelectButton(QPushButton):
    def __init__(self, text, idx, parent=None):
        super().__init__(text, parent)
        self.idx = idx
        

class SignChoiceDialog(QDialog):
    def __init__(self, sign_list):
        super().__init__()

        self.result_idx = -1
        self.label = QLabel("Choose the sign:")     
        
        layout = QGridLayout()
        layout.addWidget(self.label, 0, 0)
        
        self.btn_grp = QButtonGroup()
        self.btn_grp.setExclusive(True)
        
        for idx, sign_info in enumerate(sign_list):
            btn = SignSelectButton(sign_info[0], idx, self)
            btn.setIcon(QIcon(sign_info[1]))
            btn.setIconSize(QSize(24, 24))
            layout.addWidget(btn, idx+1, 0)
            
            self.btn_grp.addButton(btn)
            self.btn_grp.setId(btn, idx)

        self.btn_grp.buttonClicked.connect(self.on_click)

        self.setLayout(layout)
        self.show()

    def on_click(self, btn):
        self.result_idx = btn.idx
        self.close()

    def get_result(self):
        # -1 - no selection 
        return self.result_idx
