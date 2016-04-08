__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, PPM


class Ppm(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, PPM, frame, data)
        self._pubHz = '20'
        self._name = 'RiC_PPM'

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']

    def toDict(self):
        data = dict()
        data['type'] = PPM
        data['pubHz'] = self._pubHz
        data['name'] = self._name

        return data

    def saveToFile(self, file):
        file.write('PPM/publishHz: ' + self._pubHz + '\n')
        file.write('PPM/name: ' + self._name + '\n')

    def showDetails(self, items=None):
        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('Name: '), self.name)

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

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Name: '), QLabel(self._name))