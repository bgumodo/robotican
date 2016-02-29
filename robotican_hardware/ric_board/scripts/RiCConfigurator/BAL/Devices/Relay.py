__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, RELAY


class Relay(DeviceFrame):
    relayCount = 0

    def __init__(self, frame, data, relayPorts):
        DeviceFrame.__init__(self, RELAY, frame, data)
        self.mainPorts = relayPorts

        self._name = 'RiC_Relay'
        self._port = self._port = str(self.mainPorts.currentText())

    def fromDict(self, data):
        self._name = data['name']
        self._port = data['port']

    def toDict(self):
        data = dict()
        data['type'] = RELAY
        data['name'] = self._name
        data['port'] = self._port

        return data

    def findItem(self):
        for i in xrange(self.mainPorts.count()):
            if self._port == str(self.mainPorts.itemText(i)):
                return i
        return 1

    def getPort(self):
        return self._port

    def saveToFile(self, file):
        file.write('relay' + str(Relay.relayCount) + '/name: ' + self._name + '\n')
        file.write('relay' + str(Relay.relayCount) + '/port: ' + self._port + '\n')
        Relay.relayCount += 1

    def showDetails(self, items=None):
        self.relayPorts = QComboBox()
        self.relayPorts.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self.relayPorts.setCurrentIndex(self.findItem())

        self.name = QLineEdit(self._name)

        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Port: '), self.relayPorts)

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
        self._port = str(self.relayPorts.currentText())
        self.mainPorts.removeItem(self.relayPorts.currentIndex())

    def printDetails(self):
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Port: '), QLabel(self._port))

    def getName(self):
        return self._name