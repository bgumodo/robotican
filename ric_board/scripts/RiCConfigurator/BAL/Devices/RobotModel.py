import rospkg

__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, ROBOT_MODEL
from lxml.etree import Element, SubElement, XML


class RobotModel(DeviceFrame):

    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._filePath = ''
        self._filePathTmp = ''
        self._pkg = 'ric_description'
        self._pkgTmp = self._pkg

    def getName(self):
        return 'robot_model'

    def saveToFile(self, parent):
        SubElement(parent, 'param', {
          'name': 'robot_description',
          'command': "$(find xacro)/xacro.py '$(find %s)%s'" % (self._pkg, self._filePath)
        })
        SubElement(parent, 'node', {
            'name': 'robot_state_publisher',
            'pkg': 'robot_state_publisher',
            'type': 'state_publisher'
        })

    def add(self):
        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._isValid = False
            return
        if self._filePathTmp == '' or self._filePath == None:
            error = QErrorMessage()
            error.setWindowTitle("File path")
            error.showMessage("File path is empty.")
            error.exec_()
            self._isValid = False
            return
        self._isValid = True
        self._filePath = self._filePathTmp
        self._pkg = self._pkgTmp

    def showDetails(self, items=None):
        self._pkgTmp = self._pkg
        browse = QPushButton('Browse')
        self.pkg = QLineEdit(self._pkg)

        self.pkg.textChanged.connect(self.change)
        browse.clicked.connect(self.browse)

        self._frame.layout().addRow(QLabel('Package: '), self.pkg)
        self._frame.layout().addRow(QLabel('File path: '), browse)

    def change(self, text):
        self._pkgTmp = str(text)

    def browse(self):
        try:
            pkg = rospkg.RosPack().get_path(self._pkgTmp)
            filePath = str(QFileDialog.getOpenFileName(self._frame, self._frame.tr("File Path"), pkg, self._frame.tr("ALL (*.*)")))
            self._filePathTmp = "".join(filePath.rsplit(pkg))
        except:
            QMessageBox.critical(self._frame, "Error", "Can't find %s package." % self._pkgTmp)

    def fromDict(self, data):
        self._filePath = data['filePath']
        self._pkg = data['pkg']

    def toDict(self):
        data = dict()

        data['type'] = ROBOT_MODEL
        data['filePath'] = self._filePath
        data['pkg'] = self._pkg

        return data

    def printDetails(self):
        self._frame.layout().addRow(QLabel('Package: '), QLabel(self._pkg))
        self._frame.layout().addRow(QLabel('File path: '), QLabel(self._filePath))
