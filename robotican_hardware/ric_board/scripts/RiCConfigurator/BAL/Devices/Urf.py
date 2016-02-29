__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, URF


class Urf(DeviceFrame):
    urfCount = 0

    def __init__(self, frame, data, urfPorts):
        DeviceFrame.__init__(self, URF, frame, data)
        self.mainPorts = urfPorts
        self._pubHz = '50'
        self._name = 'RiC_URF'
        self._frameId = 'RiC_URF_Frame'
        self._urfType = '10'
        self._port = str(self.mainPorts.currentText())

    def fromDict(self, data):
        self._pubHz = data['pubHz']
        self._name = data['name']
        self._frameId = data['frameId']
        self._urfType = data['urfType']
        self._port = data['port']

    def toDict(self):
        data = dict()
        data['type'] = URF
        data['pubHz'] = self._pubHz
        data['name'] = self._name
        data['frameId'] = self._frameId
        data['urfType'] = self._urfType
        data['port'] = self._port

        return data

    def getCurrentUrfTypeInex(self):
        if self._urfType == '10': return 0
        return 1

    def findItem(self):
        for i in xrange(self.mainPorts.count()):
            if self._port == str(self.mainPorts.itemText(i)):
                return i
        return 1

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Publish Hz: '), QLabel(self._pubHz))
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Frame id: '), QLabel(self._frameId))
        if self._urfType == '10': self._frame.layout().addRow(QLabel('Type: '), QLabel('HRLV-MaxSonar-EZ'))
        else: self._frame.layout().addRow(QLabel('Type: '), QLabel('LV-MaxSonar-EZ'))
        self._frame.layout().addRow(QLabel('Port: '), QLabel(self._port))

    def getName(self):
        return self._name

    def getPort(self):
        return self._port

    def saveToFile(self, file):
        file.write('URF' + str(Urf.urfCount) + '/publishHz: ' + self._pubHz + '\n')
        file.write('URF' + str(Urf.urfCount) + '/name: ' + self._name + '\n')
        file.write('URF' + str(Urf.urfCount) + '/frameId: ' + self._frameId + '\n')
        file.write('URF' + str(Urf.urfCount) + '/type: ' + self._urfType + '\n')
        file.write('URF' + str(Urf.urfCount) + '/port: ' + self._port + '\n')
        Urf.urfCount += 1

    def showDetails(self, items=None):
        self.urfPorts = QComboBox()
        self.urfTypes = QComboBox()

        self.urfTypes.addItem('HRLV-MaxSonar-EZ', '10')
        self.urfTypes.addItem('LV-MaxSonar-EZ', '11')
        self.urfPorts.addItems([self.mainPorts.itemText(i) for i in xrange(self.mainPorts.count())])
        self.urfPorts.setCurrentIndex(self.findItem())
        self.urfTypes.setCurrentIndex(self.getCurrentUrfTypeInex())

        self.pubHz = QLineEdit(self._pubHz)
        self.name = QLineEdit(self._name)
        self.frameId = QLineEdit(self._frameId)
        self.frameId = QLineEdit(self._frameId)

        self._frame.layout().addRow(QLabel('Publish Hz: '), self.pubHz)
        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Frame id: '), self.frameId)
        self._frame.layout().addRow(QLabel('Type: '), self.urfTypes)
        self._frame.layout().addRow(QLabel('Port: '), self.urfPorts)

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
        self._frameId = str(self.frameId.text())
        self._urfType = str(self.urfTypes.itemData(self.urfTypes.currentIndex()).toString())
        self._port = str(self.urfPorts.currentText())
        self.mainPorts.removeItem(self.urfPorts.currentIndex())
