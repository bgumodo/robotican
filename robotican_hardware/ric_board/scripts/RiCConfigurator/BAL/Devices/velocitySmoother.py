__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, SMOOTHER
from lxml.etree import SubElement


class VelocitySmoother(DeviceFrame):
    def __init__(self,frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._diffDriverTopic = 'diff/command'
        self._smoothTopic = 'diff/smooth_command'
        self._diffDriverFeedback = 'diff/command'
        self._diffDriverOdometryFeedback = 'diff/odometry'
        self._speedLimitLinear = '0.8'
        self._speedLimitAngular = '5.4'
        self._accelerationLimitLinear = '0.3'
        self._accelerationLimitAngular = '3.5'
        self._frequency = '20.0'
        self._deceleration = '1.0'
        self._robotFeedback = '2'

    def getName(self):
        return 'velocity_smoother'

    def toDict(self):
        data = dict()

        data['type'] = SMOOTHER
        data['diffDriverTopic'] = self._diffDriverTopic
        data['smoothTopic'] = self._smoothTopic
        data['diffDriverFeedback'] = self._diffDriverFeedback
        data['diffDriverOdometryFeedback'] = self._diffDriverOdometryFeedback
        data['speedLimitLinear'] = self._speedLimitLinear
        data['speedLimitAngular'] = self._speedLimitAngular
        data['accelerationLimitLinear'] = self._accelerationLimitLinear
        data['accelerationLimitAngular'] = self._accelerationLimitAngular
        data['frequency'] = self._frequency
        data['deceleration'] = self._deceleration
        data['robotFeedback'] = self._robotFeedback

        return data

    def fromDict(self, data):

        self._diffDriverTopic = data['diffDriverTopic']
        self._smoothTopic = data['smoothTopic']
        self._diffDriverFeedback = data['diffDriverFeedback']
        self._diffDriverOdometryFeedback = data['diffDriverOdometryFeedback']
        self._speedLimitLinear = data['speedLimitLinear']
        self._speedLimitAngular = data['speedLimitAngular']
        self._accelerationLimitLinear = data['accelerationLimitLinear']
        self._accelerationLimitAngular = data['accelerationLimitAngular']
        self._frequency = data['frequency']
        self._deceleration = data['deceleration']
        self._robotFeedback = data['robotFeedback']

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return

        self._diffDriverTopic = str(self.diffDriverTopic.text())
        self._smoothTopic = str(self.smoothTopic.text())
        self._diffDriverFeedback = str(self.diffDriverFeedback.text())
        self._diffDriverOdometryFeedback = str(self.diffDriverOdometryFeedback.text())
        self._speedLimitLinear = str(self.speedLimitLinear.text())
        self._speedLimitAngular = str(self.speedLimitAngular.text())
        self._accelerationLimitLinear = str(self.accelerationLimitLinear.text())
        self._accelerationLimitAngular = str(self.accelerationLimitAngular.text())
        self._frequency = str(self.frequency.text())
        self._deceleration = str(self.deceleration.text())
        self._robotFeedback = str(self.robotFeedback.itemData(self.robotFeedback.currentIndex()).toString())
        self._isValid = True

    def showDetails(self, items=None):

        self.diffDriverTopic = QLineEdit(self._diffDriverTopic)
        self.smoothTopic = QLineEdit(self._smoothTopic)
        self.diffDriverFeedback = QLineEdit(self._diffDriverFeedback)
        self.diffDriverOdometryFeedback = QLineEdit(self._diffDriverOdometryFeedback)
        self.speedLimitLinear = QLineEdit(self._speedLimitLinear)
        self.speedLimitAngular = QLineEdit(self._speedLimitAngular)
        self.accelerationLimitLinear = QLineEdit(self._accelerationLimitLinear)
        self.accelerationLimitAngular = QLineEdit(self._accelerationLimitAngular)
        self.frequency = QLineEdit(self._frequency)
        self.deceleration = QLineEdit(self._deceleration)
        self.robotFeedback = QComboBox()

        self.robotFeedback.addItem('None', '0')
        self.robotFeedback.addItem('Odometry', '1')
        self.robotFeedback.addItem('End robot commands', '2')
        self.robotFeedback.setCurrentIndex(int(self._robotFeedback))

        self._frame.layout().addRow(QLabel('Differential drive topic: '), self.diffDriverTopic)
        self._frame.layout().addRow(QLabel('Differential drive smooth topic: '), self.smoothTopic)
        self._frame.layout().addRow(QLabel('Differential drive end robot topic: '), self.diffDriverFeedback)
        self._frame.layout().addRow(QLabel('Differential odometry topic: '), self.diffDriverOdometryFeedback)
        self._frame.layout().addRow(QLabel('Differential speed limit linear: '), self.speedLimitLinear)
        self._frame.layout().addRow(QLabel('Differential speed limit angular: '), self.speedLimitAngular)
        self._frame.layout().addRow(QLabel('Differential acceleration limit linear: '), self.accelerationLimitLinear)
        self._frame.layout().addRow(QLabel('Differential acceleration limit angular: '), self.accelerationLimitAngular)
        self._frame.layout().addRow(QLabel('Frequency rate: '), self.frequency)
        self._frame.layout().addRow(QLabel('Deceleration rate: '), self.deceleration)
        self._frame.layout().addRow(QLabel('Feedback mode: '), self.robotFeedback)

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Differential drive topic: '), QLabel(self._diffDriverTopic))
        self._frame.layout().addRow(QLabel('Differential drive smooth topic: '), QLabel(self._smoothTopic))
        self._frame.layout().addRow(QLabel('Differential drive end robot topic: '), QLabel(self._diffDriverFeedback))
        self._frame.layout().addRow(QLabel('Differential odometry topic: '), QLabel(self._diffDriverOdometryFeedback))
        self._frame.layout().addRow(QLabel('Differential speed limit linear: '), QLabel(self._speedLimitLinear))
        self._frame.layout().addRow(QLabel('Differential speed limit angular: '), QLabel(self._speedLimitAngular))
        self._frame.layout().addRow(QLabel('Differential acceleration limit linear: '), QLabel(self._accelerationLimitLinear))
        self._frame.layout().addRow(QLabel('Differential acceleration limit angular: '), QLabel(self._accelerationLimitAngular))
        self._frame.layout().addRow(QLabel('Frequency rate: '), QLabel(self._frequency))
        self._frame.layout().addRow(QLabel('Deceleration rate: '), QLabel(self._deceleration))

        robotFeedbackText = 'End robot commands'

        if self._robotFeedback == '1':
            robotFeedbackText = 'Odometry'
        elif self._robotFeedback == '0':
            robotFeedbackText = 'None'

        self._frame.layout().addRow(QLabel('Feedback mode: '), QLabel(robotFeedbackText))

    def saveToFile(self, parent):
        element = SubElement(parent, 'include', {
            'file': '$(find ric_board)/scripts/velocity_smoother.launch'
        })

        SubElement(element, 'arg', {
            'name': 'raw_cmd_vel_topic',
            'value': self._smoothTopic
        })
        SubElement(element, 'arg', {
            'name': 'smooth_cmd_vel_topic',
            'value': self._diffDriverTopic
        })
        SubElement(element, 'arg', {
            'name': 'robot_cmd_vel_topic',
            'value': self._diffDriverFeedback
        })
        SubElement(element, 'arg', {
            'name': 'odom_topic',
            'value': self._diffDriverOdometryFeedback
        })
        SubElement(element, 'arg', {
            'name': 'SPEED_LIM_V',
            'value': self._speedLimitLinear
        })
        SubElement(element, 'arg', {
            'name': 'SPEED_LIM_W',
            'value': self._speedLimitAngular
        })
        SubElement(element, 'arg', {
            'name': 'ACCEL_LIM_V',
            'value': self._accelerationLimitLinear
        })
        SubElement(element, 'arg', {
            'name': 'ACCEL_LIM_W',
            'value': self._accelerationLimitAngular
        })
        SubElement(element, 'arg', {
            'name': 'FREQUENCY',
            'value': self._frequency
        })
        SubElement(element, 'arg', {
            'name': 'DECEL_FACTOR',
            'value': self._deceleration
        })
        SubElement(element, 'arg', {
            'name': 'ROBOT_FEEDBACK',
            'value': self._robotFeedback
        })