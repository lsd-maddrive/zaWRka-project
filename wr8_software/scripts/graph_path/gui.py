from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
import sys

from . import resources as rss
from .maze import Node

SIGNS_NONE          = 0
SIGNS_NO_PATH       = 1
SIGNS_FORWARD       = 2
SIGNS_RIGHT         = 3
SIGNS_LEFT          = 4
SIGNS_FORWARD_RIGHT = 5
SIGNS_FORWARD_LEFT  = 6


sign_idx_2_meaning = [None] * 7
sign_idx_2_meaning[SIGNS_NONE]          = SIGNS_NONE
sign_idx_2_meaning[SIGNS_NO_PATH]       = SIGNS_NO_PATH
sign_idx_2_meaning[SIGNS_FORWARD]       = SIGNS_FORWARD
sign_idx_2_meaning[SIGNS_RIGHT]         = SIGNS_RIGHT
sign_idx_2_meaning[SIGNS_LEFT]          = SIGNS_LEFT
sign_idx_2_meaning[SIGNS_FORWARD_RIGHT] = SIGNS_FORWARD_RIGHT
sign_idx_2_meaning[SIGNS_FORWARD_LEFT]  = SIGNS_FORWARD_LEFT

signRemoveDirs = {
                    SIGNS_NO_PATH       : [Node.NEXT_IDX_FRWD],
                    SIGNS_FORWARD       : [Node.NEXT_IDX_RGHT, Node.NEXT_IDX_LEFT],
                    SIGNS_RIGHT         : [Node.NEXT_IDX_FRWD, Node.NEXT_IDX_LEFT],
                    SIGNS_LEFT          : [Node.NEXT_IDX_RGHT, Node.NEXT_IDX_FRWD],
                    SIGNS_FORWARD_RIGHT : [Node.NEXT_IDX_LEFT],
                    SIGNS_FORWARD_LEFT  : [Node.NEXT_IDX_RGHT],
                    SIGNS_NONE          : []
                    }


class SignsRequestWindow(QDialog):

    def __init__(self):
        super().__init__()

        self.choice = -1
        self.initUI()

    def signChanged(self):
        for i, btn in enumerate(self.signBtns):
            if btn.isChecked():
                self.choice = (i + 1)   # To fit to enumeration
                self.setBtn.setEnabled(True)

    def noSignClose(self):
        self.choice = SIGNS_NONE
        self.close()

    def exitClose(self):
        self.choice = -1
        self.close()

    def initUI(self):
        self.setWindowTitle("Signs")

        baseLayout = QGridLayout(self)
        choiceLayout = QGridLayout(None)

        pic = QPixmap(":/signs.jpg")
        label = QLabel(self)
        label.setPixmap(pic)
        
        choiceLayout.addWidget(label, 0, 0)

        btnsLayout = QVBoxLayout(None)

        self.signBtns = []
        for i in range(6):
            b_s = QRadioButton("Sign {}".format(i+1))
            self.signBtns.append(b_s)
            btnsLayout.addWidget( b_s )

            b_s.toggled.connect(self.signChanged)

        choiceLayout.addLayout(btnsLayout, 0, 1)
        baseLayout.addLayout(choiceLayout, 0, 0, 1, 3)

        btn = QPushButton("No sign", self)
        btn.clicked.connect(self.noSignClose)
        baseLayout.addWidget(btn, 1, 1)

        btn = QPushButton("Exit", self)
        btn.clicked.connect(self.exitClose)
        baseLayout.addWidget(btn, 1, 0)

        self.setBtn = QPushButton("Set", self)
        self.setBtn.setEnabled(False)
        self.setBtn.clicked.connect(self.close)
        baseLayout.addWidget(self.setBtn, 1, 2)

        self.show()

class NotificationWindow(QDialog):

    def __init__(self, msg):
        super().__init__()
        self.msg = msg

        self.initUI()

    def initUI(self):
        self.setWindowTitle("Notification")

        baseLayout = QGridLayout(self)

        label = QLabel(self.msg)

        okBtn = QPushButton("Ok", self)
        okBtn.clicked.connect(self.close)

        baseLayout.addWidget(label, 0, 0)
        baseLayout.addWidget(okBtn, 1, 0)


        self.show()

def notifyUser(msg):
   app = QApplication(sys.argv)

   req = NotificationWindow(msg)

   app.exec()


def requestSign():
   app = QApplication(sys.argv)

   req = SignsRequestWindow()

   app.exec()

   return req.choice

def test():
    res = requestSign()
    print(res)
    notifyUser('Test')
    

if __name__ == '__main__':
    test()

