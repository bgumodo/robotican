__author__ = 'tom1231'

from PyQt4.QtGui import *
from GUI.Schemes.RicBoardPiC import Ui_RiCBoard


class ShowRiCBoard(QDialog, Ui_RiCBoard):

    def __init__(self, parent=None):
        super(ShowRiCBoard, self).__init__(parent)
        self.setupUi(self)


