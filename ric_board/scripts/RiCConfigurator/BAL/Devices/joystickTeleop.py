__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, JOYSTICK
from lxml.etree import SubElement


class JoystickTeleop(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)

        self._maxSpeed = '1'
        self._maxSpeedAngular = '0.5'
        self._boost = '2'
        self._topic = 'diff/command'
        self._pubHz = '20'
        self._joystickNum = '0'
        self._upAndDownAxis = '1'
        self._leftAndRightNum = '0'
        self._boostButton = '4'
        self._driveButton = '5'
        self._upAndDownAxisDir = False
        self._leftAndRightDir = False

    def showDetails(self, items=None):
        self.maxSpeed = QLineEdit(self._maxSpeed)
        self.maxSpeedAngular = QLineEdit(self._maxSpeedAngular)
        self.boost = QLineEdit(self._boost)
        self.topic = QLineEdit(self._topic)
        self.pubHz = QLineEdit(self._pubHz)
        self.joystickNum = QLineEdit(self._joystickNum)
        self.upAndDownAxis = QLineEdit(self._upAndDownAxis)
        self.leftAndRightNum = QLineEdit(self._leftAndRightNum)
        self.boostButton = QLineEdit(self._boostButton)
        self.driveButton = QLineEdit(self._driveButton)
        self.upAndDownAxisDir = QCheckBox('')
        self.leftAndRightDir = QCheckBox('')

        self.upAndDownAxisDir.setChecked(self._upAndDownAxisDir)
        self.leftAndRightDir.setChecked(self._leftAndRightDir)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('Differential drive topic: '), self.topic)
        self._frame.layout().addRow(QLabel('Max Speed linear: '), self.maxSpeed)
        self._frame.layout().addRow(QLabel('Max Speed angular: '), self.maxSpeedAngular)
        self._frame.layout().addRow(QLabel('Scale: '), self.boost)
        self._frame.layout().addRow(QLabel('Joystick number: '), self.joystickNum)
        self._frame.layout().addRow(QLabel('Up and down axis number: '), self.upAndDownAxis)
        self._frame.layout().addRow(QLabel('Left and right axis number: '), self.leftAndRightNum)
        self._frame.layout().addRow(QLabel('Scale button  number: '), self.boostButton)
        self._frame.layout().addRow(QLabel('Enable to drive button  number: '), self.driveButton)
        self._frame.layout().addRow(QLabel('Reverse  up and down axis: '), self.upAndDownAxisDir)
        self._frame.layout().addRow(QLabel('Reverse  left and right axis: '), self.leftAndRightDir)

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return

        self._maxSpeed = str(self.maxSpeed.text())
        self._boost = str(self.boost.text())
        self._topic = str(self.topic.text())
        self._pubHz = str(self.pubHz.text())
        self._maxSpeedAngular = str(self.maxSpeedAngular.text())
        self._joystickNum = str(self.joystickNum.text())
        self._upAndDownAxis = str(self.upAndDownAxis.text())
        self._leftAndRightNum = str(self.leftAndRightNum.text())
        self._boostButton = str(self.boostButton.text())
        self._driveButton = str(self.driveButton.text())
        self._upAndDownAxisDir = self.upAndDownAxisDir.isChecked()
        self._leftAndRightDir = self.leftAndRightDir.isChecked()
        self._isValid = True

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Differential drive topic: '), QLabel(self._topic))
        self._frame.layout().addRow(QLabel('Max Speed linear: '), QLabel(self._maxSpeed))
        self._frame.layout().addRow(QLabel('Max Speed angular: '), QLabel(self._maxSpeedAngular))
        self._frame.layout().addRow(QLabel('Scale: '), QLabel(self._boost))
        self._frame.layout().addRow(QLabel('Joystick number: '), QLabel(self._joystickNum))
        self._frame.layout().addRow(QLabel('Up and down axis number: '), QLabel(self._upAndDownAxis))
        self._frame.layout().addRow(QLabel('Left and right axis number: '), QLabel(self._leftAndRightNum))
        self._frame.layout().addRow(QLabel('Scale button  number: '), QLabel(self._boostButton))
        self._frame.layout().addRow(QLabel('Enable to drive button  number: '), QLabel(self._driveButton))

        upAndDownDirText = 'Disable'
        leftAndRightDirText = 'Disable'

        if self._upAndDownAxisDir: upAndDownDirText = 'Enable'
        if self._leftAndRightDir: leftAndRightDirText = 'Enable'

        self._frame.layout().addRow(QLabel('Reverse  up and down axis: '), QLabel(upAndDownDirText))
        self._frame.layout().addRow(QLabel('Reverse  left and right axis: '), QLabel(leftAndRightDirText))

    def getName(self):
        return 'joystick_teleop'

    def toDict(self):
        data = dict()

        data['type'] = JOYSTICK
        data['maxSpeed'] = self._maxSpeed
        data['boost'] = self._boost
        data['topic'] = self._topic
        data['maxSpeedAngular'] = self._maxSpeedAngular
        data['pubHz'] = self._pubHz
        data['joystickNum'] = self._joystickNum
        data['upAndDownAxis'] = self._upAndDownAxis
        data['leftAndRightNum'] = self._leftAndRightNum
        data['driveButton'] = self._driveButton
        data['upAndDownAxisDir'] = self._upAndDownAxisDir
        data['leftAndRightDir'] = self._leftAndRightDir
        data['boostButton'] = self._boostButton

        return data

    def fromDict(self, data):
        self._maxSpeed = data['maxSpeed']
        self._boost = data['boost']
        self._topic = data['topic']

        if data.has_key('maxSpeedAngular'):
            self._maxSpeedAngular = data['maxSpeedAngular']
        if data.has_key('pubHz'):
            self._pubHz = data['pubHz']
        if data.has_key('joystickNum'):
            self._joystickNum = data['joystickNum']
        if data.has_key('upAndDownAxis'):
            self._upAndDownAxis = data['upAndDownAxis']
        if data.has_key('leftAndRightNum'):
            self._leftAndRightNum = data['leftAndRightNum']
        if data.has_key('driveButton'):
            self._driveButton = data['driveButton']
        if data.has_key('upAndDownAxisDir'):
            self._upAndDownAxisDir = data['upAndDownAxisDir']
        if data.has_key('leftAndRightDir'):
            self._leftAndRightDir = data['leftAndRightDir']
        if data.has_key('boostButton'):
            self._boostButton = data['boostButton']

    def saveToFile(self, parent):
        element = SubElement(parent, 'include', {
            'file': '$(find ric_base_station)/launch/joystick_teleop.launch'
        })

        SubElement(element, 'arg', {
            'name': 'maxspeed',
            'value': self._maxSpeed
        })

        SubElement(element, 'arg', {
            'name': 'boost',
            'value': self._boost
        })

        SubElement(element, 'arg', {
            'name': 'topic',
            'value': self._topic
        })

        SubElement(element, 'arg', {
            'name': 'pubHz',
            'value': self._pubHz
        })

        SubElement(element, 'arg', {
            'name': 'maxSpeedAngular',
            'value': self._maxSpeedAngular
        })
        SubElement(element, 'arg', {
            'name': 'LEFT_RIGHT_NUM',
            'value': self._leftAndRightNum
        })
        SubElement(element, 'arg', {
            'name': 'UP_DOWN_NUM',
            'value': self._upAndDownAxis
        })
        SubElement(element, 'arg', {
            'name': 'BOOST_SPEED_BUTTON',
            'value': self._boostButton
        })
        SubElement(element, 'arg', {
            'name': 'ENABLE_BUTTON',
            'value': self._driveButton
        })
        SubElement(element, 'arg', {
            'name': 'JOY_NUM',
            'value': self._joystickNum
        })

        upAndDownDirValue = '1'
        leftAndRightDirValue = '1'

        if self._upAndDownAxisDir: upAndDownDirValue = '-1'
        if self._leftAndRightDir: leftAndRightDirValue = '-1'

        SubElement(element, 'arg', {
            'name': 'REV_UP_DOWN',
            'value': upAndDownDirValue
        })
        SubElement(element, 'arg', {
            'name': 'REV_LEFT_RIGHT',
            'value': leftAndRightDirValue
        })