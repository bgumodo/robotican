from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EMERGENCY_SWITCH


class EmergencySwitch(DeviceFrame):
    emergency_switch_count = 0

    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EMERGENCY_SWITCH, frame, data)
        self._pin_to_listen = '27'
        self._state = '1'
        self._name = 'emergency_switch'

    def showDetails(self, items=None):
        self.pin_to_listen = QLineEdit(self._pin_to_listen)
        self.state = QComboBox()
        self.name = QLineEdit(self._name)

        self.state.addItem("Normally open", '1')
        self.state.addItem("Normally close", '2')
        self.state.setCurrentIndex(int(self._state) - 1)

        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Pin to listen: '), self.pin_to_listen)
        self._frame.layout().addRow(QLabel('Mode: '), self.state)

    def getName(self):
        return self._name

    def saveToFile(self, file):
        file.write('emergency_switch{0}/pin: {1}\n'.format(str(EmergencySwitch.emergency_switch_count), self._pin_to_listen))
        file.write('emergency_switch{0}/state: {1}\n'.format(str(EmergencySwitch.emergency_switch_count), self._state))
        file.write('emergency_switch{0}/name: {1}\n'.format(str(EmergencySwitch.emergency_switch_count), self._name))
        EmergencySwitch.emergency_switch_count += 1

    def printDetails(self):
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Pin to listen: '), QLabel(self._pin_to_listen))

        mode = 'Normally '
        if self._state == '1':
            mode += 'open'
        else:
            mode += 'close'

        self._frame.layout().addRow(QLabel('Mode: '), QLabel(mode))

    def fromDict(self, data):
        self._pin_to_listen = data['pinToListen']
        self._state = data['state']
        if data.has_key('name'):
            self._name = data['name']
        else:
            self._name = 'emergency_switch'

    def toDict(self):
        data = dict()

        data['type'] = EMERGENCY_SWITCH
        data['name'] = self.getName()
        data['pinToListen'] = self._pin_to_listen
        data['state'] = self._state

        return data

    def add(self):
        old_name = self._name
        self._name = str(self.name.text())
        if not self.nameIsValid():
            QMessageBox.critical(self._frame, "Error", "Name already exist")
            self._name = old_name
            self._isValid = False
            return
        self._pin_to_listen = str(self.pin_to_listen.text())
        self._state = str(self.state.itemData(self.state.currentIndex()).toString())
        self._isValid = True
