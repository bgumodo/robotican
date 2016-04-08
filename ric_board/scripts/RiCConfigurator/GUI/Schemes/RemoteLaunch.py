# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'RemoteLaunch.ui'
#
# Created: Thu Jul  2 12:14:58 2015
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

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName(_fromUtf8("Dialog"))
        Dialog.resize(404, 200)
        Dialog.setStyleSheet(_fromUtf8("#Dialog{\n"
"    background-color: qlineargradient(spread:pad, x1:1, y1:0.682, x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1 rgba(171, 171, 171, 255));\n"
"}\n"
""))
        self.label = QtGui.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(10, 10, 91, 41))
        self.label.setObjectName(_fromUtf8("label"))
        self.label_2 = QtGui.QLabel(Dialog)
        self.label_2.setGeometry(QtCore.QRect(10, 40, 121, 31))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(Dialog)
        self.label_3.setGeometry(QtCore.QRect(10, 70, 121, 41))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.hostIpAdd = QtGui.QLineEdit(Dialog)
        self.hostIpAdd.setGeometry(QtCore.QRect(140, 20, 131, 21))
        self.hostIpAdd.setText(_fromUtf8(""))
        self.hostIpAdd.setPlaceholderText(_fromUtf8(""))
        self.hostIpAdd.setObjectName(_fromUtf8("hostIpAdd"))
        self.hostUser = QtGui.QLineEdit(Dialog)
        self.hostUser.setGeometry(QtCore.QRect(140, 50, 131, 21))
        self.hostUser.setObjectName(_fromUtf8("hostUser"))
        self.hostPassword = QtGui.QLineEdit(Dialog)
        self.hostPassword.setGeometry(QtCore.QRect(140, 80, 131, 21))
        self.hostPassword.setText(_fromUtf8(""))
        self.hostPassword.setEchoMode(QtGui.QLineEdit.Password)
        self.hostPassword.setObjectName(_fromUtf8("hostPassword"))
        self.launchButton = QtGui.QPushButton(Dialog)
        self.launchButton.setGeometry(QtCore.QRect(277, 140, 121, 27))
        self.launchButton.setObjectName(_fromUtf8("launchButton"))
        self.label_4 = QtGui.QLabel(Dialog)
        self.label_4.setGeometry(QtCore.QRect(10, 110, 81, 20))
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.localIP = QtGui.QLineEdit(Dialog)
        self.localIP.setGeometry(QtCore.QRect(140, 110, 131, 21))
        self.localIP.setObjectName(_fromUtf8("localIP"))
        self.TestButton = QtGui.QPushButton(Dialog)
        self.TestButton.setGeometry(QtCore.QRect(280, 20, 121, 21))
        self.TestButton.setObjectName(_fromUtf8("TestButton"))
        self.path = QtGui.QLabel(Dialog)
        self.path.setGeometry(QtCore.QRect(140, 160, 121, 31))
        self.path.setTextFormat(QtCore.Qt.PlainText)
        self.path.setObjectName(_fromUtf8("path"))
        self.browse = QtGui.QPushButton(Dialog)
        self.browse.setGeometry(QtCore.QRect(0, 160, 81, 27))
        self.browse.setObjectName(_fromUtf8("browse"))
        self.newTerm = QtGui.QPushButton(Dialog)
        self.newTerm.setGeometry(QtCore.QRect(277, 170, 121, 27))
        self.newTerm.setObjectName(_fromUtf8("newTerm"))

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(_translate("Dialog", "Dialog", None))
        self.label.setToolTip(_translate("Dialog", "<html><head/><body><p>Enter the robot ip address.</p></body></html>", None))
        self.label.setText(_translate("Dialog", "Robot IP:", None))
        self.label_2.setToolTip(_translate("Dialog", "<html><head/><body><p>The robot user.</p></body></html>", None))
        self.label_2.setText(_translate("Dialog", "Robot Username: ", None))
        self.label_3.setText(_translate("Dialog", "Robot  password:", None))
        self.launchButton.setText(_translate("Dialog", "Launch", None))
        self.label_4.setText(_translate("Dialog", "Local ip :", None))
        self.TestButton.setText(_translate("Dialog", "Test connection ", None))
        self.path.setText(_translate("Dialog", "Empty File", None))
        self.browse.setText(_translate("Dialog", "Browse", None))
        self.newTerm.setText(_translate("Dialog", "Launch terminal", None))

