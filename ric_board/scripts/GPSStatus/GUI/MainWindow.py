__author__ = 'tom'
from PyQt4.QtGui import *
from GUI.Shceme.gpsDialog import Ui_gpsDialog


class MainWindow(QDialog, Ui_gpsDialog):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
