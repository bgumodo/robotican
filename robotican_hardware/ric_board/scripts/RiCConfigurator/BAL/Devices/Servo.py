from PySide import QtCore
from PyQt4.QtCore import pyqtSlot, SIGNAL

__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, SERVO


class Servo(DeviceFrame):
    servoCount = 0

    def __init__(self, frame, data, servoPorts):
        DeviceFrame.__init__(self, SERVO, frame, data)
        self._pubHz = '10'
        self._name = 'RiC_Servo'
        self._initPos = '999.0'
        self._min = '-1.57'
        self._max = '1.57'
        self._a = '90.0'
        self._b = '57.3'
        self._port = str(servoPorts.currentText())
        self._isinitPos = False
        self.mainPorts = servoPorts

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']
        self._initPos = data['initPos']
        self._isinitPos = data['isInitPos']
        self._min = data['min']
        self._max = data['max']
        self._a = data['a']
        self._b = data['b']
        self._port = data['port']

    def toDict(self):
        data = dict()

        data['type'] = SERVO
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['initPos'] = self._initPos
        data['isInitPos'] = self._isinitPos
        data['min'] = self._min
        data['max'] = self._max
        data['a'] = self._a
        data['b'] = self._b
        data['port'] = self._port
        return data

    def saveToFile(self, file):
        if not self._isinitPos: self._initPos = '999.0'

        file.write('servo' + str(Servo.servoCount) + '/publishHz: ' + self._pubHz + '\n')
        file.write('servo' + str(Servo.servoCount) + '/name: ' + self._name + '\n')
        file.write('servo' + str(Servo.servoCount) + '/port: ' + self._port + '\n')
        file.write('servo' + str(Servo.servoCount) + '/min: ' + self._min + '\n')
        file.write('servo' + str(Servo.servoCount) + '/max: ' + self._max + '\n')
        file.write('servo' + str(Servo.servoCount) + '/initMove: ' + self._initPos + '\n')
        file.write('servo' + str(Servo.servoCount) + '/a: ' + self._a + '\n')
        file.write('servo' + str(Servo.servoCount) + '/b: ' + self._b + '\n')
        Servo.servoCount += 1

    def getName(self):
        return self._name

    def getPort(self):
        return self._port

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
        self._isinitPos = self.check.isChecked()
        if self._isinitPos: self._initPos = str(self.initPos.text())
        self._min = str(self.min.text())
        self._max = str(self.max.text())
        self._a = str(self.a.text())
        self._b = str(self.b.text())
        self._port = str(self._servoPorts.currentText())
        self.mainPorts.removeItem(self._servoPorts.currentIndex())

    def findItem(self):
        for i in xrange(self.mainPorts.count()):
            if self._port == str(self.mainPorts.itemText(i)):
                return i
        return 1

    def showDetails(self, items=None):
        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)
        self.initPos = QLineEdit(self._initPos)
        self.min = QLineEdit(self._min)
        self.max = QLineEdit(self._max)
        self.a = QLineEdit(self._a)
        self.b = QLineEdit(self._b)
        self.check = QCheckBox('')
        self._servoPorts = QComboBox()
        self._servoPorts.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self._servoPorts.setCurrentIndex(self.findItem())
        self.initPos.setEnabled(self._isinitPos)
        self.check.setChecked(self._isinitPos)
        self.check.stateChanged.connect(self.initPos.setEnabled)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Initial position(in radians): '), self.initPos)
        self._frame.layout().addRow(QLabel('Minimum position(in radians): '), self.min)
        self._frame.layout().addRow(QLabel('Maximum position(in radians): '), self. max)
        self._frame.layout().addRow(QLabel('Parameter A(in degrees): '), self.a)
        self._frame.layout().addRow(QLabel('Parameter B(in degrees): '), self.b)
        self._frame.layout().addRow(QLabel('Port: '), self._servoPorts)
        self._frame.layout().addRow(QLabel('Enable initial position: '), self.check)

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        if self._isinitPos: self._frame.layout().addRow(QLabel('Initial position(in radians): '), QLabel(self._initPos))
        else: self._frame.layout().addRow(QLabel('Initial position(in radians): '), QLabel('Not enable'))
        self._frame.layout().addRow(QLabel('Minimum position(in radians): '), QLabel(self._min))
        self._frame.layout().addRow(QLabel('Maximum position(in radians): '), QLabel(self._max))
        self._frame.layout().addRow(QLabel('Parameter A(in degrees): '), QLabel(self._a))
        self._frame.layout().addRow(QLabel('Parameter B(in degrees): '), QLabel(self._b))
        self._frame.layout().addRow(QLabel('Port: '), QLabel(self._port))
