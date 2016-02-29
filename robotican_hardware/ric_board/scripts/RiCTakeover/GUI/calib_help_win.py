from PyQt4.QtGui import *
from GUI.Schemes.clibHelp import Ui_Dialog


class CalibHelp(QDialog, Ui_Dialog):
    def __init__(self, parent=None):
        super(CalibHelp, self).__init__(parent)
        self.setupUi(self)
