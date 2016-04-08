# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gpsDialog.ui'
#
# Created: Sat Aug 29 20:06:08 2015
#      by: PyQt4 UI code generator 4.10.4
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

class Ui_gpsDialog(object):
    def setupUi(self, gpsDialog):
        gpsDialog.setObjectName(_fromUtf8("gpsDialog"))
        gpsDialog.resize(595, 300)
        gpsDialog.setStyleSheet(_fromUtf8("#gpsDialog{\n"
"    background-color: rgb(0, 85, 255)\n"
"}"))
        self.gridLayout_4 = QtGui.QGridLayout(gpsDialog)
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.label_9 = QtGui.QLabel(gpsDialog)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_3.addWidget(self.label_9, 0, 0, 1, 1)
        self.label_10 = QtGui.QLabel(gpsDialog)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout_3.addWidget(self.label_10, 0, 1, 1, 1)
        self.label_11 = QtGui.QLabel(gpsDialog)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_3.addWidget(self.label_11, 1, 0, 1, 1)
        self.label_12 = QtGui.QLabel(gpsDialog)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_3.addWidget(self.label_12, 1, 1, 1, 1)
        self.verticalLayout_2.addLayout(self.gridLayout_3)
        self.label_7 = QtGui.QLabel(gpsDialog)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.verticalLayout_2.addWidget(self.label_7)
        self.label_8 = QtGui.QLabel(gpsDialog)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.verticalLayout_2.addWidget(self.label_8)
        self.gridLayout_4.addLayout(self.verticalLayout_2, 1, 1, 1, 1)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label_3 = QtGui.QLabel(gpsDialog)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout.addWidget(self.label_3, 1, 0, 1, 1)
        self.label_4 = QtGui.QLabel(gpsDialog)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout.addWidget(self.label_4, 0, 0, 1, 1)
        self.label_5 = QtGui.QLabel(gpsDialog)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout.addWidget(self.label_5, 0, 1, 1, 1)
        self.label_6 = QtGui.QLabel(gpsDialog)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout.addWidget(self.label_6, 1, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.label = QtGui.QLabel(gpsDialog)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.label_2 = QtGui.QLabel(gpsDialog)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.verticalLayout.addWidget(self.label_2)
        self.gridLayout_4.addLayout(self.verticalLayout, 1, 0, 1, 1)
        self.label_13 = QtGui.QLabel(gpsDialog)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_4.addWidget(self.label_13, 0, 0, 1, 1)
        self.status = QtGui.QLabel(gpsDialog)
        self.status.setObjectName(_fromUtf8("status"))
        self.gridLayout_4.addWidget(self.status, 0, 1, 1, 1)

        self.retranslateUi(gpsDialog)
        QtCore.QMetaObject.connectSlotsByName(gpsDialog)

    def retranslateUi(self, gpsDialog):
        gpsDialog.setWindowTitle(_translate("gpsDialog", "Gps status", None))
        self.label_9.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">Roll (deg)</span></p></body></html>", None))
        self.label_10.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">Pitch (deg)</span></p></body></html>", None))
        self.label_11.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">0.0</span></p></body></html>", None))
        self.label_12.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">0.0</span></p></body></html>", None))
        self.label_7.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">Yaw (deg)</span></p></body></html>", None))
        self.label_8.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">0.0</span></p></body></html>", None))
        self.label_3.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">0.0 </span><span style=\" font-size:16pt; color:#ff0000; vertical-align:super;\">0</span><span style=\" font-size:16pt; color:#ff0000;\">N</span></p></body></html>", None))
        self.label_4.setText(_translate("gpsDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#ff0000;\">Latitude</span></p></body></html>", None))
        self.label_5.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">Longitude</span></p></body></html>", None))
        self.label_6.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">0.0 </span><span style=\" font-size:16pt; color:#ff0000; vertical-align:super;\">0</span><span style=\" font-size:16pt; color:#ff0000;\">E</span></p></body></html>", None))
        self.label.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">GPS Fix</span></p></body></html>", None))
        self.label_2.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt; color:#ff0000;\">-1.0</span></p></body></html>", None))
        self.label_13.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:18pt; color:#ff0000;\">GPS status: </span></p></body></html>", None))
        self.status.setText(_translate("gpsDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:20pt; color:#ff0000;\">Not connected!</span></p></body></html>", None))

