__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, SWITCH


class Switch(DeviceFrame):
    switchCount = 0

    def __init__(self, frame, data, switchPorts):
        DeviceFrame.__init__(self, SWITCH, frame, data)
        self.mainPorts = switchPorts

        self._pubHz = '20'
        self._name = 'RiC_Switch'
        self._port = str(self.mainPorts.currentText())

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']
        self._port = data['port']

    def toDict(self):
        data = dict()

        data['type'] = SWITCH
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['port'] = self._port

        return data

    def saveToFile(self, file):
        file.write('switch' + str(Switch.switchCount) + '/name: ' + self._name + '\n')
        file.write('switch' + str(Switch.switchCount) + '/port: ' + self._port + '\n')
        file.write('switch' + str(Switch.switchCount) + '/publishHz: ' + self._pubHz + '\n')
        Switch.switchCount += 1

    def getPort(self):
        return self._port

    def findItem(self):
        for i in xrange(self.mainPorts.count()):
            if self._port == str(self.mainPorts.itemText(i)):
                return i
        return 1

    def showDetails(self, items=None):
        self.switchPorts = QComboBox()
        self.switchPorts.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self.switchPorts.setCurrentIndex(self.findItem())

        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Port: '), self.switchPorts)


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

        self._isValid = True
        self._pubHz = str(self.pubHz.text())
        self._name = str(self.name.text())
        self._port = str(self.switchPorts.currentText())
        self.mainPorts.removeItem(self.switchPorts.currentIndex())

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('Name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Port: '), QLabel(self._port))
