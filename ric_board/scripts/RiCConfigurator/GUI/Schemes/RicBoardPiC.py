# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'RicBoardPiC.ui'
#
# Created: Wed Jun 10 09:46:54 2015
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

class Ui_RiCBoard(object):
    def setupUi(self, RiCBoard):
        RiCBoard.setObjectName(_fromUtf8("RiCBoard"))
        RiCBoard.resize(700, 553)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(RiCBoard.sizePolicy().hasHeightForWidth())
        RiCBoard.setSizePolicy(sizePolicy)
        RiCBoard.setMinimumSize(QtCore.QSize(700, 553))
        RiCBoard.setMaximumSize(QtCore.QSize(700, 553))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        RiCBoard.setWindowIcon(icon)
        RiCBoard.setStyleSheet(_fromUtf8("QDialog {\n"
"        background-color: qlineargradient(spread:pad, x1:1, y1:0.682, x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1 rgba(171, 171, 171, 255));\n"
"    background-image: url(:/images/gui.png);\n"
"}"))

        self.retranslateUi(RiCBoard)
        QtCore.QMetaObject.connectSlotsByName(RiCBoard)

    def retranslateUi(self, RiCBoard):
        RiCBoard.setWindowTitle(_translate("RiCBoard", "RiC Board Ports", None))

import resource_rc
