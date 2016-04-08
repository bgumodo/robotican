#!/usr/bin/env python

__author__ = 'tom1231'
import rospy
from GUI.MainWindow import MainWindow
import sys
from PyQt4.QtGui import *

def main():
    rospy.init_node('RiC_SetParam')
    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()
