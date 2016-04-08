__author__ = 'tom1231'

from PyQt4.QtCore import QUrl
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, PPMReader
from lxml.etree import Element, SubElement, XML

class PPMReader(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._diffTopic = '/diff'
        self._ppmTopic = '/RiC_PPM'

    def fromDict(self, data):
        self._diffTopic = data['diff']
        self._ppmTopic = data['ppm']

    def toDict(self):
        data = dict()

        data['type'] = PPMReader
        data['diff'] = self._diffTopic
        data['ppm'] = self._ppmTopic

        return data

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return
        self._diffTopic = str(self.diffTopic.text())
        self._ppmTopic = str(self.ppmTopic.text())
        self._isValid = True

    def showDetails(self, items=None):
        self.diffTopic = QLineEdit(self._diffTopic)
        self.ppmTopic = QLineEdit(self._ppmTopic)

        self._frame.layout().addRow(QLabel('Differential drive topic: '), self.diffTopic)
        self._frame.layout().addRow(QLabel('PPM topic: '), self.ppmTopic)

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Differential drive topic: '), QLabel(self._diffTopic))
        self._frame.layout().addRow(QLabel('PPM topic: '), QLabel(self._ppmTopic))

    def saveToFile(self, parent):
        keysAtt = parent.keys()
        ns = ''
        if len(keysAtt) > 0 and keysAtt[0] == 'ns':
            ns = '/' + parent.get('ns')
        element = SubElement(parent, 'include', {
            'file': '$(find ric_board)/scripts/RiCPPMReader.launch'
        })

        SubElement(element, 'arg', {
            'name': 'ppmTopic',
            'value': ns + self._ppmTopic
        })

        SubElement(element, 'arg', {
            'name': 'diffTopic',
            'value': ns + self._diffTopic
        })

    def getName(self):
        return 'ppm_reader'

