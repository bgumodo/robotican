__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, BATTERY


class Battery(DeviceFrame):

    def __init__(self, frame, data):
        DeviceFrame.__init__(self, BATTERY, frame, data)
        self._pubHz = '10'
        self._voltDivRation = '6'

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._voltDivRation = data['voltDivRation']

    def toDict(self):
        data = dict()
        data['type'] = BATTERY
        data['pubHz'] = self._pubHz
        data['voltDivRation'] = self._voltDivRation

        return data

    def saveToFile(self, file):
        file.write('Battery/name: ' + self.getName() + '\n')
        file.write('Battery/pubHz: ' + self._pubHz + '\n')
        file.write('Battery/voltageDividerRatio: ' + self._voltDivRation + '\n')

    def showDetails(self, items=None):
        self.puHzField = QLineEdit(self._pubHz)
        self.voltDivRationField = QLineEdit(self._voltDivRation)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.puHzField)
        self._frame.layout().addRow(QLabel('Voltage divider ratio: '), self.voltDivRationField)

    def getName(self):
        return 'battery_monitor'

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return
        self._isValid = True
        self._pubHz = str(self.puHzField.text())
        self._voltDivRation = str(self.voltDivRationField.text())

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Voltage divider ratio: '), QLabel(self._voltDivRation))
