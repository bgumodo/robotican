# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'UsbRoles.ui'
#
# Created: Wed Jun 10 09:17:48 2015
#      by: PyQt4 UI code generator 4.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_UsbRoles(object):
    def setupUi(self, UsbRoles):
        UsbRoles.setObjectName(_fromUtf8("UsbRoles"))
        UsbRoles.resize(210, 92)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        UsbRoles.setWindowIcon(icon)
        UsbRoles.setStyleSheet(_fromUtf8("QDialog {\n"
"        background-color: qlineargradient(spread:pad, x1:1, y1:0.682, x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1 rgba(171, 171, 171, 255));\n"
"}"))
        self.buttonBox = QtGui.QDialogButtonBox(UsbRoles)
        self.buttonBox.setGeometry(QtCore.QRect(20, 60, 181, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName(_fromUtf8("buttonBox"))
        self.password = QtGui.QLineEdit(UsbRoles)
        self.password.setGeometry(QtCore.QRect(80, 10, 121, 21))
        self.password.setEchoMode(QtGui.QLineEdit.Password)
        self.password.setObjectName(_fromUtf8("password"))
        self.label = QtGui.QLabel(UsbRoles)
        self.label.setGeometry(QtCore.QRect(10, 0, 81, 41))
        self.label.setObjectName(_fromUtf8("label"))

        self.retranslateUi(UsbRoles)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("accepted()")), UsbRoles.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL(_fromUtf8("rejected()")), UsbRoles.reject)
        QtCore.QMetaObject.connectSlotsByName(UsbRoles)

    def retranslateUi(self, UsbRoles):
        UsbRoles.setWindowTitle(_translate("UsbRoles", "USB Roles", None))
        self.password.setPlaceholderText(_translate("UsbRoles", "Enter password", None))
        self.label.setText(_translate("UsbRoles", "Password:", None))

import resource_rc
