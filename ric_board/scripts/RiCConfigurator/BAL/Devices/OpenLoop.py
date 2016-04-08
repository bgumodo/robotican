__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, OPEN_LOP


class OpenLoop(DeviceFrame):
    openLoopNum = 0

    def __init__(self, frame, data):
        DeviceFrame.__init__(self, OPEN_LOP, frame, data)

        self._name = 'motor'
        self._driverAdd = '128'
        self._channel = '1'
        self._timeout = '1000'
        self._maxSpeed = '127'
        self._dirMotor = False

    def fromDict(self, data):

        self._name = data['name']
        self._driverAdd = data['driverAdd']
        self._channel = data['channel']
        self._timeout = data['timeout']
        self._maxSpeed = data['maxSpeed']
        self._dirMotor = data['dirMotor']

    def toDict(self):
        data = dict()
        data['type'] = OPEN_LOP
        data['name'] = self._name
        data['driverAdd'] = self._driverAdd
        data['channel'] = self._channel
        data['timeout'] = self._timeout
        data['maxSpeed'] = self._maxSpeed
        data['dirMotor'] = self._dirMotor

        return data

    def saveToFile(self, file):
        file.write('openLoop' + str(OpenLoop.openLoopNum) + '/name: ' + self._name + '\n')
        file.write('openLoop' + str(OpenLoop.openLoopNum) + '/address: ' + self._driverAdd + '\n')
        file.write('openLoop' + str(OpenLoop.openLoopNum) + '/channel: ' + self._channel + '\n')
        file.write('openLoop' + str(OpenLoop.openLoopNum) + '/timeout: ' + self._timeout + '\n')
        file.write('openLoop' + str(OpenLoop.openLoopNum) + '/max: ' + self._maxSpeed + '\n')
        if self._dirMotor: file.write('openLoop' + str(OpenLoop.openLoopNum) + '/direction: ' + '-1' + '\n')
        else: file.write('openLoop' + str(OpenLoop.openLoopNum) + '/direction: ' + '1' + '\n')
        OpenLoop.openLoopNum += 1

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
        self._driverAdd = str(self.driverAdd.text())
        self._channel = str(self.channel.text())
        self._timeout = str(self.timeout.text())
        self._maxSpeed = str(self.maxSpeed.text())
        self._dirMotor = self.dirMotor.isChecked()

    def printDetails(self):
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Driver Address: '), QLabel(self._driverAdd))
        self._frame.layout().addRow(QLabel('Channel: '), QLabel(self._channel))
        self._frame.layout().addRow(QLabel('Timeout (in millisSecond): '), QLabel(self._timeout))
        self._frame.layout().addRow(QLabel('Max speed: '), QLabel(self._maxSpeed))
        if self._dirMotor: self._frame.layout().addRow(QLabel('Direction: '), QLabel('Enable'))
        else: self._frame.layout().addRow(QLabel('Reverse direction: '), QLabel('Disable'))

    def getName(self):
        return self._name

    def showDetails(self, items=None):
        self.name = QLineEdit(self._name)
        self.driverAdd = QLineEdit(self._driverAdd)
        self.channel = QLineEdit(self._channel)
        self.timeout = QLineEdit(self._timeout)
        self.maxSpeed = QLineEdit(self._maxSpeed)
        self.dirMotor = QCheckBox('')

        self.dirMotor.setChecked(self._dirMotor)

        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Driver Address: '), self.driverAdd)
        self._frame.layout().addRow(QLabel('Channel: '), self.channel)
        self._frame.layout().addRow(QLabel('Timeout (in millisSecond): '), self.timeout)
        self._frame.layout().addRow(QLabel('Max speed: '), self.maxSpeed)
        self._frame.layout().addRow(QLabel('Reverse direction: '), self.dirMotor)