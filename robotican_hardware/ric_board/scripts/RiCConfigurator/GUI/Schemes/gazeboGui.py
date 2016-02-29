# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gazeboGui.ui'
#
# Created: Wed Aug 26 11:06:18 2015
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

class Ui_gazebo_gui(object):
    def setupUi(self, gazebo_gui):
        gazebo_gui.setObjectName(_fromUtf8("gazebo_gui"))
        gazebo_gui.resize(276, 393)
        gazebo_gui.setMinimumSize(QtCore.QSize(276, 393))
        gazebo_gui.setMaximumSize(QtCore.QSize(276, 393))
        gazebo_gui.setStyleSheet(_fromUtf8("#gazebo_gui {    \n"
"    background-color: \n"
"        qlineargradient(spread:pad, x1:1, y1:0.682, x2:0.966825, y2:0, stop:0  \n"
"        rgba(224, 224, 224, 255), stop:1 rgba(171, 171, 171, 255));\n"
"}"))
        self.verticalLayoutWidget_2 = QtGui.QWidget(gazebo_gui)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 10, 251, 331))
        self.verticalLayoutWidget_2.setObjectName(_fromUtf8("verticalLayoutWidget_2"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setSpacing(15)
        self.verticalLayout_2.setMargin(0)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.nameSpaceLabel = QtGui.QLabel(self.verticalLayoutWidget_2)
        self.nameSpaceLabel.setObjectName(_fromUtf8("nameSpaceLabel"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.nameSpaceLabel)
        self.nameSpaceLineEdit = QtGui.QLineEdit(self.verticalLayoutWidget_2)
        self.nameSpaceLineEdit.setObjectName(_fromUtf8("nameSpaceLineEdit"))
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.nameSpaceLineEdit)
        self.numberOfRobotsLabel = QtGui.QLabel(self.verticalLayoutWidget_2)
        self.numberOfRobotsLabel.setObjectName(_fromUtf8("numberOfRobotsLabel"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.numberOfRobotsLabel)
        self.numberOfRobotsSpinBox = QtGui.QSpinBox(self.verticalLayoutWidget_2)
        self.numberOfRobotsSpinBox.setMinimum(1)
        self.numberOfRobotsSpinBox.setMaximum(10)
        self.numberOfRobotsSpinBox.setObjectName(_fromUtf8("numberOfRobotsSpinBox"))
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.numberOfRobotsSpinBox)
        self.verticalLayout_2.addLayout(self.formLayout)
        self.robotComboBox = QtGui.QComboBox(self.verticalLayoutWidget_2)
        self.robotComboBox.setObjectName(_fromUtf8("robotComboBox"))
        self.robotComboBox.addItem(_fromUtf8(""))
        self.robotComboBox.addItem(_fromUtf8(""))
        self.robotComboBox.addItem(_fromUtf8(""))
        self.verticalLayout_2.addWidget(self.robotComboBox)
        self.label = QtGui.QLabel(self.verticalLayoutWidget_2)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout_2.addWidget(self.label)
        self.devList = QtGui.QListWidget(self.verticalLayoutWidget_2)
        self.devList.setFrameShape(QtGui.QFrame.StyledPanel)
        self.devList.setFrameShadow(QtGui.QFrame.Sunken)
        self.devList.setObjectName(_fromUtf8("devList"))
        self.verticalLayout_2.addWidget(self.devList)
        self.launchButton = QtGui.QPushButton(gazebo_gui)
        self.launchButton.setGeometry(QtCore.QRect(140, 350, 121, 27))
        self.launchButton.setObjectName(_fromUtf8("launchButton"))
        self.loadButton = QtGui.QPushButton(gazebo_gui)
        self.loadButton.setGeometry(QtCore.QRect(10, 350, 111, 27))
        self.loadButton.setObjectName(_fromUtf8("loadButton"))

        self.retranslateUi(gazebo_gui)
        QtCore.QMetaObject.connectSlotsByName(gazebo_gui)

    def retranslateUi(self, gazebo_gui):
        gazebo_gui.setWindowTitle(_translate("gazebo_gui", "gazebo build", None))
        self.nameSpaceLabel.setText(_translate("gazebo_gui", "Name Space:", None))
        self.numberOfRobotsLabel.setText(_translate("gazebo_gui", "Number of robots:", None))
        self.robotComboBox.setToolTip(_translate("gazebo_gui", "<html><head/><body><p>Select robot </p></body></html>", None))
        self.robotComboBox.setItemText(0, _translate("gazebo_gui", "Select robot", None))
        self.robotComboBox.setItemText(1, _translate("gazebo_gui", "Komodo", None))
        self.robotComboBox.setItemText(2, _translate("gazebo_gui", "Lizi", None))
        self.label.setText(_translate("gazebo_gui", "<html><head/><body><p><span style=\" font-size:14pt;\">Robot details</span></p></body></html>", None))
        self.launchButton.setText(_translate("gazebo_gui", "launch gazebo", None))
        self.loadButton.setText(_translate("gazebo_gui", "Load new file", None))

