__author__ = 'tom1231'
from PyQt4.QtCore import QUrl
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, HOKUYO
from lxml.etree import Element, SubElement, XML


class Hokuyo(DeviceFrame):
    def __init__(self, frame, data=None):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._name = 'Hokuyo_Node'
        self._output = 'screen'
        self._port = '/dev/Hokuyo'
        self._frameId = 'Hokuyo_Frame'

    def onLink(self, URL):
        QDesktopServices().openUrl(QUrl(URL))

    def fromDict(self, data):
        self._name = data['name']
        self._output = data['output']
        self._port = data['port']
        self._frameId = data['frameId']

    def toDict(self):
        data = dict()

        data['type'] = HOKUYO
        data['name'] = self._name
        data['output'] = self._output
        data['port'] = self._port
        data['frameId'] = self._frameId

        return data

    def printDetails(self):
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Output: '), QLabel(self._output))
        self._frame.layout().addRow(QLabel('Port: '), QLabel(self._port))
        self._frame.layout().addRow(QLabel('Frame id: '), QLabel(self._frameId))

    def getName(self):
        return self._name

    def showDetails(self, items=None):
        self.name = QLineEdit(self._name)
        self.output = QLineEdit(self._output)
        self.port = QLineEdit(self._port)
        self.frameId = QLineEdit(self._frameId)

        link = QLabel("<a href = http://wiki.ros.org/hokuyo_node> Hokuyo Wiki </a>")
        link.linkActivated.connect(self.onLink)

        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Output: '), self.output)
        self._frame.layout().addRow(QLabel('Port: '), self.port)
        self._frame.layout().addRow(QLabel('Frame id: '), self.frameId)
        self._frame.layout().addRow(QLabel('More information: '), link)

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

        self._isValid = True
        self._name = str(self.name.text())
        self._output = str(self.output.text())
        self._port = str(self.port.text())
        self._frameId = str(self.frameId.text())

    def saveToFile(self, parent):
        element = SubElement(parent, 'node', {
            'pkg': 'hokuyo_node',
            'type': 'hokuyo_node',
            'name': self._name,
            'output': self._output
        })
        SubElement(element, 'param', {
            'name': 'port',
            'value': self._port
        })
        SubElement(element, 'param', {
            'name': 'frame_id',
            'value': self._frameId
        })