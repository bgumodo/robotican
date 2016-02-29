__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, DIFF_CLOSE_FOUR


class DiffCloseFour(DeviceFrame):

    def __init__(self, frame, data, motors):
        DeviceFrame.__init__(self, DIFF_CLOSE_FOUR, frame, data)
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
        self._motorFL = '0'
        self._motorFR = '1'
        self._motorBL = '2'
        self._motorBR = '3'

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
        self._motorFL = data['motorFL']
        self._motorFR = data['motorFR']
        self._motorBL = data['motorBL']
        self._motorBR = data['motorBR']

    def add(self):
        old = self._name
        self._name = str(self.name.text())
        self._motorFL = str(self.motorsFL.itemData(self.motorsFL.currentIndex()).toString())
        self._motorFR = str(self.motorsFR.itemData(self.motorsFR.currentIndex()).toString())
        self._motorBL = str(self.motorsBL.itemData(self.motorsBL.currentIndex()).toString())
        self._motorBR = str(self.motorsBR.itemData(self.motorsBR.currentIndex()).toString())

        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._name = old
            self._isValid = False
            return

        if self._motorFL == self._motorFR or self._motorBL == self._motorBR or self._motorFL == self._motorBL or self._motorFL == self._motorBR or self._motorFR == self._motorBL or self._motorFR == self._motorBR:
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
        self._frame.layout().addRow(QLabel('Motor front left: '), QLabel(self.motors[int(self._motorFL)]))
        self._frame.layout().addRow(QLabel('Motor front right: '), QLabel(self.motors[int(self._motorFR)]))
        self._frame.layout().addRow(QLabel('Motor back left: '), QLabel(self.motors[int(self._motorBL)]))
        self._frame.layout().addRow(QLabel('Motor back right: '), QLabel(self.motors[int(self._motorBR)]))


    def showDetails(self, items=None):
        self.motorsFL = QComboBox()
        self.motorsFR = QComboBox()
        self.motorsBL = QComboBox()
        self.motorsBR = QComboBox()

        for i in xrange(len(self.motors)):
            self.motorsFL.addItem(self.motors[i], str(i))
            self.motorsFR.addItem(self.motors[i], str(i))
            self.motorsBL.addItem(self.motors[i], str(i))
            self.motorsBR.addItem(self.motors[i], str(i))
        self.motorsFL.setCurrentIndex(int(self._motorFL))
        self.motorsFR.setCurrentIndex(int(self._motorFR))
        self.motorsBL.setCurrentIndex(int(self._motorBL))
        self.motorsBR.setCurrentIndex(int(self._motorBR))
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
        self._frame.layout().addRow(self.motorsFL, self.motorsFR)
        self._frame.layout().addRow(self.motorsBL, self.motorsBR)

    def getName(self):
        return self._name

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
        file.write('Diff/motorFL: ' + self._motorFL + '\n')
        file.write('Diff/motorFR: ' + self._motorFR + '\n')
        file.write('Diff/motorBL: ' + self._motorBL + '\n')
        file.write('Diff/motorBR: ' + self._motorBR + '\n')

    def toDict(self):
        data = dict()

        data['type'] = DIFF_CLOSE_FOUR
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['rWheel'] = self._rWheel
        data['width'] = self._width
        data['base'] = self._base
        data['odom'] = self._odom
        data['slip'] = self._slip
        data['maxAg'] = self._maxAg
        data['maxLn'] = self._maxLn
        data['motorFL'] = self._motorFL
        data['motorFR'] = self._motorFR
        data['motorBL'] = self._motorBL
        data['motorBR'] = self._motorBR

        return data