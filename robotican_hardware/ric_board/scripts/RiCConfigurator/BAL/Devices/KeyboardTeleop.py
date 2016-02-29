__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, Keyboard
from lxml.etree import SubElement


class KeyboardTeleop(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._cmd = 'cmd_val'

    def getName(self):
        return 'keyboardTel'

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return
        self._cmd = str(self.cmd.text())
        self._isValid = True

    def showDetails(self, items=None):
        self.cmd = QLineEdit(self._cmd)

        self._frame.layout().addRow(QLabel('Differential drive name: '), self.cmd)

    def fromDict(self, data):
        self._cmd = data['cmd']

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Differential drive name: '), QLabel(self._cmd))

    def saveToFile(self, parent):
        element = SubElement(parent, 'include', {
            'file': '$(find ric_base_station)/launch/keyboard_teleop.launch'
        })
        SubElement(element, 'arg', {
            'name': 'topic',
            'value': self._cmd
        })

    def toDict(self):
        data = dict()

        data['type'] = Keyboard
        data['cmd'] = self._cmd

        return data