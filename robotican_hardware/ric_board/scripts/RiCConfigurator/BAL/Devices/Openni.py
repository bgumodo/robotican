__author__ = 'tom1231'
from PyQt4.QtCore import QUrl
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, OPRNNI
from lxml.etree import Element, SubElement, XML


class Opennni(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._value = 'None'

    def fromDict(self, data):
        self._value = data['name']

    def toDict(self):
        data = dict()

        data['type'] = OPRNNI
        data['name'] = self._value

        return data

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Value: '), QLabel(self._value))

    def getName(self):
        return 'OpenniCamera'

    def showDetails(self, items=None):
        self.value = QLineEdit(self._value)
        link = QLabel("<a href = http://wiki.ros.org/openni2_launch> Openni Wiki </a>")
        link.linkActivated.connect(self.onLink)

        self._frame.layout().addRow(QLabel('Value: '), self.value)
        self._frame.layout().addRow(QLabel('More information: '), link)

    def onLink(self, URL):
        QDesktopServices().openUrl(QUrl(URL))

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return
        self._isValid = True
        self._value = str(self.value.text())

    def saveToFile(self, parent):
        element = SubElement(parent, 'include', {
            'file': '$(find openni2_launch)/launch/openni2.launch'
        })
        SubElement(element, 'arg', {
            'name': 'camera',
            'value': self._value
        })