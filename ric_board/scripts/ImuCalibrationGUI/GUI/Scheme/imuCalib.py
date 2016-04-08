# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'imuCalib.ui'
#
# Created: Mon Aug 24 17:26:35 2015
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

class Ui_imuCalib(object):
    def setupUi(self, imuCalib):
        imuCalib.setObjectName(_fromUtf8("imuCalib"))
        imuCalib.resize(324, 292)
        imuCalib.setMinimumSize(QtCore.QSize(324, 292))
        imuCalib.setMaximumSize(QtCore.QSize(324, 292))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/images/icon.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        imuCalib.setWindowIcon(icon)
        imuCalib.setStyleSheet(_fromUtf8("#imuCalib{\n"
"    background-color: qlineargradient(spread:pad, x1:1, y1:0.682,             x2:0.966825, y2:0, stop:0 rgba(224, 224, 224, 255), stop:1                         rgba(171, 171, 171, 255));\n"
"}"))
        self.formLayoutWidget = QtGui.QWidget(imuCalib)
        self.formLayoutWidget.setGeometry(QtCore.QRect(20, 40, 271, 223))
        self.formLayoutWidget.setObjectName(_fromUtf8("formLayoutWidget"))
        self.formLayout = QtGui.QFormLayout(self.formLayoutWidget)
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setMargin(0)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.maxXLabel = QtGui.QLabel(self.formLayoutWidget)
        self.maxXLabel.setObjectName(_fromUtf8("maxXLabel"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.maxXLabel)
        self.maxXLineEdit = QtGui.QLineEdit(self.formLayoutWidget)
        self.maxXLineEdit.setObjectName(_fromUtf8("maxXLineEdit"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.maxXLineEdit)
        self.minXLabel = QtGui.QLabel(self.formLayoutWidget)
        self.minXLabel.setObjectName(_fromUtf8("minXLabel"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.minXLabel)
        self.minXLineEdit = QtGui.QLineEdit(self.formLayoutWidget)
        self.minXLineEdit.setObjectName(_fromUtf8("minXLineEdit"))
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.minXLineEdit)
        self.maxYLabel = QtGui.QLabel(self.formLayoutWidget)
        self.maxYLabel.setObjectName(_fromUtf8("maxYLabel"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.maxYLabel)
        self.minYLabel = QtGui.QLabel(self.formLayoutWidget)
        self.minYLabel.setObjectName(_fromUtf8("minYLabel"))
        self.formLayout.setWidget(4, QtGui.QFormLayout.LabelRole, self.minYLabel)
        self.minYLineEdit = QtGui.QLineEdit(self.formLayoutWidget)
        self.minYLineEdit.setText(_fromUtf8(""))
        self.minYLineEdit.setObjectName(_fromUtf8("minYLineEdit"))
        self.formLayout.setWidget(4, QtGui.QFormLayout.FieldRole, self.minYLineEdit)
        self.maxZLabel = QtGui.QLabel(self.formLayoutWidget)
        self.maxZLabel.setObjectName(_fromUtf8("maxZLabel"))
        self.formLayout.setWidget(5, QtGui.QFormLayout.LabelRole, self.maxZLabel)
        self.maxZLineEdit = QtGui.QLineEdit(self.formLayoutWidget)
        self.maxZLineEdit.setObjectName(_fromUtf8("maxZLineEdit"))
        self.formLayout.setWidget(5, QtGui.QFormLayout.FieldRole, self.maxZLineEdit)
        self.minZLabel = QtGui.QLabel(self.formLayoutWidget)
        self.minZLabel.setObjectName(_fromUtf8("minZLabel"))
        self.formLayout.setWidget(6, QtGui.QFormLayout.LabelRole, self.minZLabel)
        self.minZLineEdit = QtGui.QLineEdit(self.formLayoutWidget)
        self.minZLineEdit.setObjectName(_fromUtf8("minZLineEdit"))
        self.formLayout.setWidget(6, QtGui.QFormLayout.FieldRole, self.minZLineEdit)
        self.maxYLineEdit = QtGui.QLineEdit(self.formLayoutWidget)
        self.maxYLineEdit.setObjectName(_fromUtf8("maxYLineEdit"))
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.maxYLineEdit)
        self.calibButton = QtGui.QPushButton(imuCalib)
        self.calibButton.setEnabled(True)
        self.calibButton.setGeometry(QtCore.QRect(20, 262, 281, 27))
        self.calibButton.setMinimumSize(QtCore.QSize(281, 27))
        self.calibButton.setMaximumSize(QtCore.QSize(281, 27))
        self.calibButton.setAutoDefault(True)
        self.calibButton.setObjectName(_fromUtf8("calibButton"))
        self.label = QtGui.QLabel(imuCalib)
        self.label.setGeometry(QtCore.QRect(20, 0, 66, 31))
        self.label.setObjectName(_fromUtf8("label"))
        self.status = QtGui.QLabel(imuCalib)
        self.status.setGeometry(QtCore.QRect(60, 0, 141, 31))
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        self.status.setObjectName(_fromUtf8("status"))
        self.refreshButton = QtGui.QPushButton(imuCalib)
        self.refreshButton.setGeometry(QtCore.QRect(200, 0, 91, 27))
        self.refreshButton.setObjectName(_fromUtf8("refreshButton"))

        self.retranslateUi(imuCalib)
        QtCore.QMetaObject.connectSlotsByName(imuCalib)
        imuCalib.setTabOrder(self.maxXLineEdit, self.minXLineEdit)
        imuCalib.setTabOrder(self.minXLineEdit, self.maxYLineEdit)
        imuCalib.setTabOrder(self.maxYLineEdit, self.minYLineEdit)
        imuCalib.setTabOrder(self.minYLineEdit, self.maxZLineEdit)
        imuCalib.setTabOrder(self.maxZLineEdit, self.minZLineEdit)
        imuCalib.setTabOrder(self.minZLineEdit, self.calibButton)

    def retranslateUi(self, imuCalib):
        imuCalib.setWindowTitle(_translate("imuCalib", "IMU Calibration", None))
        self.maxXLabel.setText(_translate("imuCalib", "Max x:", None))
        self.minXLabel.setText(_translate("imuCalib", "Min x:", None))
        self.maxYLabel.setText(_translate("imuCalib", "Max y:", None))
        self.minYLabel.setText(_translate("imuCalib", "Min y:", None))
        self.maxZLabel.setText(_translate("imuCalib", "Max z:", None))
        self.minZLabel.setText(_translate("imuCalib", "min z:", None))
        self.calibButton.setText(_translate("imuCalib", "Start calibration", None))
        self.label.setText(_translate("imuCalib", "Status:", None))
        self.status.setText(_translate("imuCalib", "Not connected", None))
        self.refreshButton.setText(_translate("imuCalib", "Refresh", None))

import resource_rc
