__author__ = 'tom1231'
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from GUI.Schemes.UsbRoles import Ui_UsbRoles


class UsbRolesDialog(QDialog, Ui_UsbRoles):

    def __init__(self, parent=None):
        super(UsbRolesDialog, self).__init__(parent)
        self.setupUi(self)
        self.fined = pyqtSignal(int)



