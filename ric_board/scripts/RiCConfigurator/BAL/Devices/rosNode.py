__author__ = 'tom'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import NODE, EX_DEV, DeviceFrame
from lxml.etree import Element, SubElement, XML


class RosNode(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._name = ''
        self._output = 'screen'
        self._pkg = ''
        self._nodeType = ''

    def getName(self):
        return self._name

    def toDict(self):
        data = dict()

        data['type'] = NODE
        data['name'] = self._name
        data['output'] = self._output
        data['pkg'] = self._pkg
        data['nodeType'] = self._nodeType

        return data

    def fromDict(self, data):
        self._name = data['name']
        self._output = data['output']
        self._pkg = data['pkg']
        self._nodeType = data['nodeType']

    def showDetails(self, items=None):
        self.name = QLineEdit(self._name)
        self.output = QLineEdit(self._output)
        self.pkg = QLineEdit(self._pkg)
        self.nodeType = QLineEdit(self._nodeType)

        self._frame.layout().addRow(QLabel('Name:'), self.name)
        self._frame.layout().addRow(QLabel('Output:'), self.output)
        self._frame.layout().addRow(QLabel('package:'), self.pkg)
        self._frame.layout().addRow(QLabel('Type:'), self.nodeType)

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Name:'), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Output:'), QLabel(self._output))
        self._frame.layout().addRow(QLabel('package:'), QLabel(self._pkg))
        self._frame.layout().addRow(QLabel('Type:'), QLabel(self._nodeType))

    def add(self):
        old = self._name
        self._name = str(self.name.text())

        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._name = old
            self._isValid = False
            return

        self._output = str(self.output.text())
        self._pkg = str(self.pkg.text())
        self._nodeType = str(self.nodeType.text())
        self._isValid = True

    def saveToFile(self, parent):
        SubElement(parent, 'node', dict(name=self._name, output=self._output, pkg=self._pkg, type=self._nodeType))
