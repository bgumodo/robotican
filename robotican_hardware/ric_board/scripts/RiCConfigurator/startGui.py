#!/usr/bin/env python
__author__ = 'tom1231'
from PyQt4.QtGui import *
from GUI.MainWindow import MainWindow
import sys
import rospy


def main():
    rospy.init_node('RiC_GUI')
    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()
