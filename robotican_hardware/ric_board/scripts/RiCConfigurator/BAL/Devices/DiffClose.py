__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, DIFF_CLOSE


class DiffClose(DeviceFrame):
    def __init__(self, frame, data, motors):
        DeviceFrame.__init__(self, DIFF_CLOSE, frame, data)
        self.motors = motors
        self._pubHz = '50'
        self._name = 'diff_driver'
        self._rWheel = '0.065'
        self._width = '0.255'
        self._base = 'base_link'
        self._odom = 'odom_link'
        self._slip = '0.85'
        self._maxAg = '16.0'
        self._maxLn = '16.0'
        self._motorL = '0'
        self._motorR = '1'

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']
        self._rWheel = data['rWheel']
        self._width = data['width']
        self._base = data['base']
        self._odom = data['odom']
        self._slip = data['slip']
        self._maxAg = data['maxAg']
        self._maxLn = data['maxLn']
        self._motorL = data['motorL']
        self._motorR = data['motorR']

    def toDict(self):
        data = dict()

        data['type'] = DIFF_CLOSE
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['rWheel'] = self._rWheel
        data['width'] = self._width
        data['base'] = self._base
        data['odom'] = self._odom
        data['slip'] = self._slip
        data['maxAg'] = self._maxAg
        data['maxLn'] = self._maxLn
        data['motorL'] = self._motorL
        data['motorR'] = self._motorR

        return data

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Radius of the wheel (in meters): '), QLabel(self._rWheel))
        self._frame.layout().addRow(QLabel('Width of the robot (in meters): '), QLabel(self._width))
        self._frame.layout().addRow(QLabel('Base Link name: '), QLabel(self._base))
        self._frame.layout().addRow(QLabel('Odometry name: '), QLabel(self._odom))
        self._frame.layout().addRow(QLabel('Slip factor: '), QLabel(self._slip))
        self._frame.layout().addRow(QLabel('Max angular: '), QLabel(self._maxAg))
        self._frame.layout().addRow(QLabel('Max linear: '), QLabel(self._maxLn))
        self._frame.layout().addRow(QLabel('MotorL: '), QLabel(self.motors[int(self._motorL)]))
        self._frame.layout().addRow(QLabel('MotorR: '), QLabel(self.motors[int(self._motorR)]))

    def getName(self):
        return self._name

    def showDetails(self, items=None):
        self.motorsL = QComboBox()
        self.motorsR = QComboBox()

        for i in xrange(len(self.motors)):
            self.motorsL.addItem(self.motors[i], str(i))
            self.motorsR.addItem(self.motors[i], str(i))
        self.motorsL.setCurrentIndex(int(self._motorL))
        self.motorsR.setCurrentIndex(int(self._motorR))
        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)
        self.rWheel = QLineEdit(self._rWheel)
        self.width = QLineEdit(self._width)
        self.base = QLineEdit(self._base)
        self.odom = QLineEdit(self._odom)
        self.slip = QLineEdit(self._slip)
        self.maxAg = QLineEdit(self._maxAg)
        self.maxLn = QLineEdit(self._maxLn)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('Name: '), self.name)
        self._frame.layout().addRow(QLabel('Radius of the wheel (in meters): '), self.rWheel)
        self._frame.layout().addRow(QLabel('Width of the robot (in meters): '), self.width)
        self._frame.layout().addRow(QLabel('Base Link name: '), self.base)
        self._frame.layout().addRow(QLabel('Odometry name: '), self.odom)
        self._frame.layout().addRow(QLabel('Slip factor: '), self.slip)
        self._frame.layout().addRow(QLabel('Max angular: '), self.maxAg)
        self._frame.layout().addRow(QLabel('Max linear: '), self.maxLn)
        self._frame.layout().addRow(self.motorsL, self.motorsR)

    def add(self):
        old = self._name
        self._name = str(self.name.text())
        self._motorL = str(self.motorsL.itemData(self.motorsL.currentIndex()).toString())
        self._motorR = str(self.motorsR.itemData(self.motorsR.currentIndex()).toString())

        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._name = old
            self._isValid = False
            return

        if self._motorL == self._motorR:
            error = QErrorMessage()
            error.setWindowTitle(" Error")
            error.showMessage("Can not have the same motor in the right and left fields.")
            error.exec_()
            self._isValid = False
            return

        self._isValid = True
        self._pubHz = str(self.pubHz.text())
        self._name = str(self.name.text())
        self._rWheel = str(self.rWheel.text())
        self._width = str(self.width.text())
        self._base = str(self.base.text())
        self._odom = str(self.odom.text())
        self._slip = str(self.slip.text())
        self._maxAg = str(self.maxAg.text())
        self._maxLn = str(self.maxLn.text())

    def saveToFile(self, file):
        file.write('Diff/publishHz: ' + self._pubHz + '\n')
        file.write('Diff/name: ' + self._name + '\n')
        file.write('Diff/rWheel: ' + self._rWheel + '\n')
        file.write('Diff/width: ' + self._width + '\n')
        file.write('Diff/baseLink: ' + self._base + '\n')
        file.write('Diff/odom: ' + self._odom + '\n')
        file.write('Diff/slip: ' + self._slip + '\n')
        file.write('Diff/maxAng: ' + self._maxAg + '\n')
        file.write('Diff/maxLin: ' + self._maxLn + '\n')
        file.write('Diff/motorL: ' + self._motorL + '\n')
        file.write('Diff/motorR: ' + self._motorR + '\n')