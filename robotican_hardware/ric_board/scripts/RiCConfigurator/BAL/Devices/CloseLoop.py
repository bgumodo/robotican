__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, CLOSE_LOP_ONE


class CloseLoop(DeviceFrame):
    closeLoop = 0

    def __init__(self, frame, data, encoders):
        DeviceFrame.__init__(self, CLOSE_LOP_ONE, frame, data)
        self.mainPorts = encoders

        self._pubHz = '20'
        self._name = 'motor'
        self._lpfHz = '50'
        self._lpfAlpha = '0.7'
        self._driverAdd = '128'
        self._channel = '1'
        self._pidHz = '1000'
        self._kp = '3.0'
        self._ki = '3.0'
        self._kd = '0'
        self._limit = '1.0'
        self._maxSpeed = '16.0'
        self._ppr = '4480'
        self._timeout = '1000'
        self.mainPorts.setCurrentIndex(0)
        self._encoder = str(self.mainPorts.currentText())
        self._motorType = '1'
        self._dirMotor = False
        self._dirEnc = False
        self._driverType = '1'

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']
        self._lpfHz = data['lpfHz']
        self._lpfAlpha = data['lpfAlpha']
        self._driverAdd = data['driverAdd']
        self._channel = data['channel']
        self._pidHz = data['pidHz']
        self._kp = data['kp']
        self._ki = data['ki']
        self._kd = data['kd']
        self._limit = data['limit']
        self._maxSpeed = data['maxSpeed']
        self._ppr = data['ppr']
        self._timeout = data['timeout']
        self._encoder = data['encoder']
        self._motorType = data['motorType']
        self._dirMotor = data['dirMotor']
        self._dirEnc = data['dirEnc']
        self._driverType = data['driverType']

    def toDict(self):
        data = dict()
        data['type'] = CLOSE_LOP_ONE
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['lpfHz'] = self._lpfHz
        data['lpfAlpha'] = self._lpfAlpha
        data['driverAdd'] = self._driverAdd
        data['channel'] = self._channel
        data['pidHz'] = self._pidHz
        data['kp'] = self._kp
        data['ki'] = self._ki
        data['kd'] = self._kd
        data['limit'] = self._limit
        data['maxSpeed'] = self._maxSpeed
        data['ppr'] = self._ppr
        data['timeout'] = self._timeout
        data['encoder'] = self._encoder
        data['motorType'] = self._motorType
        data['dirMotor'] = self._dirMotor
        data['dirEnc'] = self._dirEnc
        data['driverType'] = self._driverType

        return data

    def getTypeIndex(self):
        if self._motorType == '1': return 1
        return 0

    def getDriverTypeIndex(self):
        if self._driverType == '1': return 0
        return 1

    def findItem(self):
        for i in xrange(self.mainPorts.count()):
            if self._encoder == str(self.mainPorts.itemText(i)):
                return i
        return 1

    def getEncoder(self):
        return self._encoder

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
        self._dirEnc = self.dirEnc.isChecked()
        self._dirMotor = self.dirMotor.isChecked()
        self._pubHz = str(self.pubHz.text())
        self._name = str(self.name.text())
        self._lpfHz = str(self.lpfHz.text())
        self._lpfAlpha = str(self.lpfAlpha.text())
        self._driverAdd = str(self.driverAdd.text())
        self._channel = str(self.channel.text())
        self._encoder = str(self.encoders.currentText())
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

        self.mainPorts.removeItem(self.encoders.currentIndex())

    def saveToFile(self, file):
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/publishHz: ' + self._pubHz + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/name: ' + self._name + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/LPFAlpha: ' + self._lpfAlpha + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/LPFHz: ' + self._lpfHz + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/driverAddress: ' + self._driverAdd + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/channel: ' + self._channel + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/port: ' + self._encoder + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/PIDHz: ' + self._pidHz + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/kP: ' + self._kp + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/kI: ' + self._ki + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/kD: ' + self._kd + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/limit: ' + self._limit + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/maxSpeed: ' + self._maxSpeed + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/cpr: ' + self._ppr + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/timeout: ' + self._timeout + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/motorType: ' + self._motorType + '\n')
        file.write('closeLoop' + str(CloseLoop.closeLoop) + '/encoderType: ' + self.encoderType() + '\n')
        if not self._dirMotor: file.write('closeLoop' + str(CloseLoop.closeLoop) + '/direction: ' + '1' + '\n')
        else: file.write('closeLoop' + str(CloseLoop.closeLoop) + '/direction: ' + '-1' + '\n')
        if not self._dirEnc: file.write('closeLoop' + str(CloseLoop.closeLoop) + '/directionE: ' + '1' + '\n')
        else: file.write('closeLoop' + str(CloseLoop.closeLoop) + '/directionE: ' + '-1' + '\n')
        CloseLoop.closeLoop += 1

    def getName(self):
        return self._name

    def printDetails(self):
        show = 'Channel: '
        if self._driverType == '2': show = 'Pin: '
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('LPF HZ: '), QLabel(self._lpfHz))
        self._frame.layout().addRow(QLabel('LPF Alpha: '), QLabel(self._lpfAlpha))
        if self._driverType == '1':
            self._frame.layout().addRow(QLabel('Driver Address: '), QLabel(self._driverAdd))
        self._frame.layout().addRow(QLabel(show), QLabel(self._channel))
        self._frame.layout().addRow(QLabel('PID Hz: '), QLabel(self._pidHz))
        self._frame.layout().addRow(QLabel('Kp parameter: '), QLabel(self._kp))
        self._frame.layout().addRow(QLabel('Ki parameter: '), QLabel(self._ki))
        self._frame.layout().addRow(QLabel('Kd parameter: '), QLabel(self._kd))
        self._frame.layout().addRow(QLabel('Integral limit: '), QLabel(self._limit))
        self._frame.layout().addRow(QLabel('Max speed: '), QLabel(self._maxSpeed))
        self._frame.layout().addRow(QLabel('PPR parameter: '), QLabel(self._ppr))
        self._frame.layout().addRow(QLabel('Timeout (in milliSecond): '), QLabel(self._timeout))
        self.printEncoder()
        if self._motorType == '1': self._frame.layout().addRow(QLabel('Type: '), QLabel('Speed'))
        else: self._frame.layout().addRow(QLabel('Type: '), QLabel('Position'))
        if self._dirMotor: self._frame.layout().addRow(QLabel('Reverse motor direction: '), QLabel('Enable'))
        else: self._frame.layout().addRow(QLabel('Reverse motor direction: '), QLabel('Disable'))
        if self._dirEnc: self._frame.layout().addRow(QLabel('Reverse encoder direction: '), QLabel('Enable'))
        else: self._frame.layout().addRow(QLabel('Reverse encoder direction: '), QLabel('Disable'))

    def showDetails(self, items=None):
        self.motorTypes = QComboBox()
        self.driverType = QComboBox()

        self.motorTypes.addItem('Position', '0')
        self.motorTypes.addItem('Speed', '1')

        self.driverType.addItem('Sabertooth Serial', '1')
        self.driverType.addItem('RC Servo', '2')

        self.driverType.setCurrentIndex(self.getDriverTypeIndex())
        self.motorTypes.setCurrentIndex(self.getTypeIndex())

        self.driverType.currentIndexChanged.connect(self.driverTypeChangeEven)

        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)
        self.lpfHz = QLineEdit(self._lpfHz)
        self.lpfAlpha = QLineEdit(self._lpfAlpha)
        self.driverAdd = QLineEdit(self._driverAdd)
        self.channel = QLineEdit(self._channel)
        self.pidHz = QLineEdit(self._pidHz)
        self.kp = QLineEdit(self._kp)
        self.ki = QLineEdit(self._ki)
        self.kd = QLineEdit(self._kd)
        self.limit = QLineEdit(self._limit)
        self.maxSpeed = QLineEdit(self._maxSpeed)
        self.ppr = QLineEdit(self._ppr)
        self.timeout = QLineEdit(self._timeout)
        self.dirMotor = QCheckBox('')
        self.dirEnc = QCheckBox('')
        if self._driverType == '1': self.channelLabel = QLabel('Channel: ')
        else: self.channelLabel = QLabel('Pin: ')

        self.dirMotor.setChecked(self._dirMotor)
        self.dirEnc.setChecked(self._dirEnc)

        self.driverTypeChangeEven(self.driverType.currentIndex())

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('LPF HZ: '), self.lpfHz)
        self._frame.layout().addRow(QLabel('LPF Alpha: '), self.lpfAlpha)
        self._frame.layout().addRow(QLabel('Driver Address: '), self.driverAdd)
        self._frame.layout().addRow(self.channelLabel, self.channel)
        self._frame.layout().addRow(QLabel('PID Hz: '), self.pidHz)
        self._frame.layout().addRow(QLabel('Kp parameter: '), self.kp)
        self._frame.layout().addRow(QLabel('Ki parameter: '), self.ki)
        self._frame.layout().addRow(QLabel('Kd parameter: '), self.kd)
        self._frame.layout().addRow(QLabel('Integral limit: '), self.limit)
        self._frame.layout().addRow(QLabel('Max speed: '), self.maxSpeed)
        self._frame.layout().addRow(QLabel('PPR parameter: '), self.ppr)
        self._frame.layout().addRow(QLabel('Timeout (in milliSecond): '), self.timeout)
        self.setEncode()
        self._frame.layout().addRow(QLabel('Type: '), self.motorTypes)
        self._frame.layout().addRow(QLabel('Reverse motor direction: '), self.dirMotor)
        self._frame.layout().addRow(QLabel('Reverse encoder direction: '), self.dirEnc)
        self._frame.layout().addRow(QLabel('Type: '), self.driverType)

    def setEncode(self):
        self.encoders = QComboBox()
        self.encoders.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self.encoders.setCurrentIndex(self.findItem())

        self._frame.layout().addRow(QLabel('Encoder: '), self.encoders)

    def printEncoder(self):
        self._frame.layout().addRow(QLabel('Encoder: '), QLabel(self._encoder))

    def encoderType(self):
        return '1'

    def driverTypeChangeEven(self, index):
        if index == 0:
            self.driverAdd.setEnabled(True)
            self.driverAdd.setEchoMode(QLineEdit.Normal)
            self.driverAdd.setText('128')
            self.channelLabel.setText('Channel: ')
            return
        self.driverAdd.setEnabled(False)
        self.driverAdd.setEchoMode(QLineEdit.NoEcho)
        self.driverAdd.setText('999')
        self.channelLabel.setText('Pin: ')

