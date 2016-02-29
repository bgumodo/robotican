# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'SetParams.ui'
#
# Created: Tue Aug  4 10:58:18 2015
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

class Ui_main(object):
    def setupUi(self, main):
        main.setObjectName(_fromUtf8("main"))
        main.resize(507, 265)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        main.setWindowIcon(icon)
        main.setStyleSheet(_fromUtf8("QWidget#main{\n"
"    background-color: qlineargradient(spread:pad, x1:1, y1:0.682, x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1 rgba(171, 171, 171, 255));\n"
"}"))
        self.devList = QtGui.QListWidget(main)
        self.devList.setGeometry(QtCore.QRect(10, 30, 221, 192))
        self.devList.setObjectName(_fromUtf8("devList"))
        self.label = QtGui.QLabel(main)
        self.label.setGeometry(QtCore.QRect(15, 10, 121, 20))
        self.label.setObjectName(_fromUtf8("label"))
        self.devFrame = QtGui.QFrame(main)
        self.devFrame.setGeometry(QtCore.QRect(270, 30, 221, 191))
        self.devFrame.setFrameShape(QtGui.QFrame.WinPanel)
        self.devFrame.setFrameShadow(QtGui.QFrame.Raised)
        self.devFrame.setObjectName(_fromUtf8("devFrame"))
        self.formLayout = QtGui.QFormLayout(self.devFrame)
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.label_2 = QtGui.QLabel(main)
        self.label_2.setGeometry(QtCore.QRect(270, 10, 61, 21))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.status = QtGui.QLabel(main)
        self.status.setGeometry(QtCore.QRect(320, 10, 111, 20))
        self.status.setFrameShadow(QtGui.QFrame.Plain)
        self.status.setTextFormat(QtCore.Qt.AutoText)
        self.status.setIndent(5)
        self.status.setObjectName(_fromUtf8("status"))
        self.setB = QtGui.QPushButton(main)
        self.setB.setGeometry(QtCore.QRect(267, 230, 221, 27))
        self.setB.setObjectName(_fromUtf8("setB"))
        self.refreshB = QtGui.QPushButton(main)
        self.refreshB.setGeometry(QtCore.QRect(50, 230, 131, 27))
        self.refreshB.setObjectName(_fromUtf8("refreshB"))

        self.retranslateUi(main)
        QtCore.QMetaObject.connectSlotsByName(main)

    def retranslateUi(self, main):
        main.setWindowTitle(_translate("main", "Set Params", None))
        self.label.setText(_translate("main", "Devices", None))
        self.label_2.setText(_translate("main", "Status: ", None))
        self.status.setText(_translate("main", "Not Connected", None))
        self.setB.setText(_translate("main", "Set parameters", None))
        self.setB.setShortcut(_translate("main", "Ctrl+S", None))
        self.refreshB.setText(_translate("main", "Refresh", None))
        self.refreshB.setShortcut(_translate("main", "F5", None))

import resource_rc
