__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, GPS


class Gps(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, GPS, frame, data)
        self._pubHz = '10'
        self._name = 'RiC_GPS'
        self._frameId = 'GPS_Frame'
        self._baudRate = '9600'

    def fromDict(self, data):

        self._pubHz = data['pubHz']
        self._name = data['name']
        self._frameId = data['frameId']
        self._baudRate = data['baudRate']

    def toDict(self):
        data = dict()

        data['type'] = GPS
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['frameId'] = self._frameId
        data['baudRate'] = self._baudRate

        return data

    def saveToFile(self, file):
        file.write('GPS/publishHz: ' + self._pubHz + '\n')
        file.write('GPS/name: ' + self._name + '\n')
        file.write('GPS/frameId: ' + self._frameId + '\n')
        file.write('GPS/baudrate: ' + self._baudRate + '\n')

    def showDetails(self, items=None):
        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)
        self.frameId = QLineEdit(self._frameId)
        self.baudRate = QLineEdit(self._baudRate)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('Name: '), self.name)
        self._frame.layout().addRow(QLabel('Frame id: '), self.frameId)
        self._frame.layout().addRow(QLabel('Baud rate: '), self.baudRate)

    def getName(self):
        return self._name

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
        self._pubHz = str(self.pubHz.text())
        self._name = str(self.name.text())
        self._frameId = str(self.frameId.text())
        self._baudRate = str(self.baudRate.text())

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Frame id: '), QLabel(self._frameId))
        self._frame.layout().addRow(QLabel('Baud rate: '), QLabel(self._baudRate))