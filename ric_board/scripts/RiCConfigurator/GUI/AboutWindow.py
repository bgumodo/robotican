__author__ = 'tom'
from GUI.Schemes.about import Ui_about
from PyQt4.QtGui import *


class About(QDialog, Ui_about):
    def __init__(self, parent=None):
        super(About, self).__init__(parent)
        self.setupUi(self)
