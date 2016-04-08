__author__ = 'tom1231'
from BAL.Interface.DeviceFrame import DeviceFrame, IMU
from PyQt4.QtGui import *



class Imu(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, IMU, frame, data)
        self._pubHz = '50'
        self._name = 'RiC_IMU'
        self._frameId = 'RiC_Frame'
        self._decAng = '0'
        self._orientation = '0'
        self._fusionHz = '10'
        self._enableFuseGyro = False


    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']
        self._frameId = data['frameId']
        self._decAng = data['decAng']
        if data.has_key('orientation'):
            self._orientation = data['orientation']
        if data.has_key('fusionHz'):
            self._fusionHz = data['fusionHz']
        if data.has_key('enableFuseGyro'):
            self._enableFuseGyro = data['enableFuseGyro']

    def toDict(self):
        data = dict()

        data['type'] = IMU
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['frameId'] = self._frameId
        data['decAng'] = self._decAng
        data['orientation'] = self._orientation
        data['fusionHz'] = self._fusionHz
        data['enableFuseGyro'] = self._enableFuseGyro

        return data

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Frame id: '), QLabel(self._frameId))
        self._frame.layout().addRow(QLabel('Declination angle: '), QLabel(self._decAng))
        self._frame.layout().addRow(QLabel('Board orientation: '), QLabel(self._orientation))
        self._frame.layout().addRow(QLabel('Fusion Hz: '), QLabel(self._fusionHz))

        isFuseGyro = 'Disable'
        if self._enableFuseGyro: isFuseGyro = 'Enable'

        self._frame.layout().addRow(QLabel('Fusion gyro: '), QLabel(isFuseGyro))

    def saveToFile(self, file):
        enableFuseGyro = '0'
        if self._enableFuseGyro:
            enableFuseGyro = '1'

        file.write('IMU/publishHz: ' + self._pubHz + '\n')
        file.write('IMU/name: ' + self._name + '\n')
        file.write('IMU/frameId: ' + self._frameId + '\n')
        file.write('IMU/camp: ' + self._decAng + '\n')
        file.write('IMU/orientation: ' + self._orientation + '\n')
        file.write('IMU/fusionHz: ' + self._fusionHz + '\n')
        file.write('IMU/fuseGyro: ' + enableFuseGyro + '\n')

    def showDetails(self, items=None):
        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)
        self.frameId = QLineEdit(self._frameId)
        self.decAng = QLineEdit(self._decAng)
        self.orientation = QLineEdit(self._orientation)
        self.fusionHz = QLineEdit(self._fusionHz)
        self.enableFuseGyro = QCheckBox('')

        self.enableFuseGyro.setChecked(self._enableFuseGyro)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('Name: '), self.name)
        self._frame.layout().addRow(QLabel('Frame id: '), self.frameId)
        self._frame.layout().addRow(QLabel('Declination angle: '), self.decAng)
        self._frame.layout().addRow(QLabel('Board orientation: '), self.orientation)
        self._frame.layout().addRow(QLabel('Fusion Hz: '), self.fusionHz)
        self._frame.layout().addRow(QLabel('Fusion gyro: '), self.enableFuseGyro)

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

        self._pubHz = str(self.pubHz.text())
        self._name = str(self.name.text())
        self._frameId = str(self.frameId.text())
        self._decAng = str(self.decAng.text())
        self._orientation = str(self.orientation.text())
        self._fusionHz = str(self.fusionHz.text())
        self._enableFuseGyro = self.enableFuseGyro.isChecked()

        if int(self._pubHz) > int(self._fusionHz):
            QMessageBox.critical(self._frame, "Error", "Fusion Hz must be equal or bigger then publish Hz.")
            return

        self._isValid = True
