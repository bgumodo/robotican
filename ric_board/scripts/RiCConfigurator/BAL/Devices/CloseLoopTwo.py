

__author__ = 'tom1231'
from BAL.Interface.DeviceFrame import CLOSE_LOP_TWO
from BAL.Devices.CloseLoop import CloseLoop
from PyQt4.QtGui import *


class CloseLoopTwo(CloseLoop):
    def __init__(self, frame, data, encoders):
        CloseLoop.__init__(self, frame, data, encoders)
        self.setType(CLOSE_LOP_TWO)
        self.mainPorts.setCurrentIndex(1)
        self._encoder2 = str(self.mainPorts.currentText())

    def getEncoders(self):
        return self._encoder, self._encoder2

    def findItem2(self):
        for i in xrange(self.mainPorts.count()):
            if self._encoder2 == str(self.mainPorts.itemText(i)):
                return i
        return 1

    def toDict(self):
        data = CloseLoop.toDict(self)
        data['type'] = CLOSE_LOP_TWO
        data['encoder2'] = self._encoder2
        return data

    def fromDict(self, data):
        CloseLoop.fromDict(self, data)
        self._encoder2 = data['encoder2']

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

        self._encoder = str(self.encoders.currentText())
        self._encoder2 = str(self.encoders2.currentText())
        if self._encoder == self._encoder2:
            error = QErrorMessage()
            error.setWindowTitle("Same encoder port error")
            error.showMessage("Can not have the same encoder ports.")
            error.exec_()
            self._isValid = False
            return
        self._isValid = True
        self._dirEnc = self.dirEnc.isChecked()
        self._dirMotor = self.dirMotor.isChecked()
        self._pubHz = str(self.pubHz.text())
        self._name = str(self.name.text())
        self._lpfHz = str(self.lpfHz.text())
        self._lpfAlpha = str(self.lpfAlpha.text())
        self._driverAdd = str(self.driverAdd.text())
        self._channel = str(self.channel.text())
        self._pidHz = str(self.pidHz.text())
        self._kp = str(self.kp.text())
        self._ki = str(self.ki.text())
        self._kd = str(self.kd.text())
        self._limit = str(self.limit.text())
        self._maxSpeed = str(self.maxSpeed.text())
        self._ppr = str(self.ppr.text())
        self._timeout = str(self.timeout.text())
        self._motorType = str(self.motorTypes.itemData(self.motorTypes.currentIndex()).toString())
        self._driverType = str(self.driverType.itemData(self.driverType.currentIndex()).toString())

        self.mainPorts.removeItem(self.findItem())

        self.mainPorts.removeItem(self.findItem2())

    def saveToFile(self, file):
        CloseLoop.saveToFile(self, file)
        file.write('closeLoop' + str(CloseLoop.closeLoop - 1) + '/port2: ' + self._encoder2 + '\n')

    def encoderType(self):
        return '2'


    def printEncoder(self):
        self._frame.layout().addRow(QLabel('Encoder1: '), QLabel(self._encoder))
        self._frame.layout().addRow(QLabel('Encoder2: '), QLabel(self._encoder2))

    def setEncode(self):
        self.encoders = QComboBox()
        self.encoders.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self.encoders.setCurrentIndex(self.findItem())

        self.encoders2 = QComboBox()
        self.encoders2.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self.encoders2.setCurrentIndex(self.findItem2())

        self._frame.layout().addRow(QLabel('Encoder1: '), self.encoders)
        self._frame.layout().addRow(QLabel('Encoder2: '), self.encoders2)


