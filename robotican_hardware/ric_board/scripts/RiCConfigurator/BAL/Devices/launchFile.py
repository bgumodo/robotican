__author__ = 'tom'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import LAUNCH, EX_DEV, DeviceFrame
from lxml.etree import Element, SubElement, XML
import rospkg


class RosLaunch(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._pkg = ''
        self._file = ''
        self._name = ''

    def toDict(self):
        data = dict()

        data['type'] = LAUNCH
        data['name'] = self._name
        data['pkg'] = self._pkg
        data['file'] = self._file

        return data

    def fromDict(self, data):
        self._name = data['name']
        self._pkg = data['pkg']
        self._file = data['file']

    def showDetails(self, items=None):
        self.name = QLineEdit(self._name)
        self.pkg = QLineEdit(self._pkg)
        self.file = QPushButton('Browse')

        self.file.clicked.connect(self.browse)

        self._frame.layout().addRow(QLabel('Name:'), self.name)
        self._frame.layout().addRow(QLabel('package:'), self.pkg)
        self._frame.layout().addRow(QLabel('Launch file:'), self.file)

    def browse(self):
        if str(self.pkg.text()) == '':
            QMessageBox.critical(self._frame, "Error", "You need to fill the package field before you browse for the file.")
            return
        try:
            fullPath = rospkg.RosPack().get_path(str(self.pkg.text()))
        except rospkg.ResourceNotFound:
            QMessageBox.critical(self._frame, "Error", "Package not found.")
            return

        launchFile = str(QFileDialog.getOpenFileName(self._frame, 'Load file', fullPath, 'Launch file (*.launch)'))
        if launchFile == '': return

        index = launchFile.find(str(self.pkg.text()))
        if index == -1:
            QMessageBox.critical(self._frame, "Error", "launch is not from %s package." % str(self.pkg.text()))
            return

        self._file = launchFile[index + len(str(self.pkg.text())):]
        self._pkg = str(self.pkg.text())

    def saveToFile(self, parent):
        SubElement(parent, 'include', {
            'file': '$(find %s)%s' % (self._pkg, self._file)
        })

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Name:'), QLabel(self._name))
        self._frame.layout().addRow(QLabel('package:'), QLabel(self._pkg))
        self._frame.layout().addRow(QLabel('launch file:'), QLabel(self._file))

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

        if self._file == '' or self._pkg == '':
            QMessageBox.critical(self._frame, "Error", "one or more field are empty please fill theme.")
            return

        self._isValid = True
