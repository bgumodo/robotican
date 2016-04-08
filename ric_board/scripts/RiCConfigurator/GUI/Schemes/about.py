# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'about.ui'
#
# Created: Thu Aug 20 12:27:29 2015
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

class Ui_about(object):
    def setupUi(self, about):
        about.setObjectName(_fromUtf8("about"))
        about.resize(610, 83)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(about.sizePolicy().hasHeightForWidth())
        about.setSizePolicy(sizePolicy)
        about.setMinimumSize(QtCore.QSize(610, 83))
        about.setMaximumSize(QtCore.QSize(610, 83))
        about.setStyleSheet(_fromUtf8("QDialog{\n"
"    background-color: qlineargradient(spread:pad, x1:1, y1:0.682,             x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1                         rgba(171, 171, 171, 255));\n"
"}"))
        self.aboutText = QtGui.QTextEdit(about)
        self.aboutText.setEnabled(True)
        self.aboutText.setGeometry(QtCore.QRect(0, 0, 611, 151))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.aboutText.sizePolicy().hasHeightForWidth())
        self.aboutText.setSizePolicy(sizePolicy)
        self.aboutText.setStyleSheet(_fromUtf8("QTextEdit{\n"
"    background-color: qlineargradient(spread:pad, x1:1, y1:0.682,             x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1                         rgba(171, 171, 171, 255));\n"
"}"))
        self.aboutText.setTabChangesFocus(False)
        self.aboutText.setUndoRedoEnabled(False)
        self.aboutText.setReadOnly(True)
        self.aboutText.setAcceptRichText(True)
        self.aboutText.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.aboutText.setObjectName(_fromUtf8("aboutText"))

        self.retranslateUi(about)
        QtCore.QMetaObject.connectSlotsByName(about)

    def retranslateUi(self, about):
        about.setWindowTitle(_translate("about", "About", None))
        self.aboutText.setHtml(_translate("about", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">RIC configurator is a tool design for working with the \'RiCBoard\'.</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">With this tool you can configure the \'RiCBoard\' parameters to work for your needs.</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Also this tool will manage the external packages which work alongside  with \'RiCBoard\'</p></body></html>", None))

