#!/usr/bin/env python

from GUI.mainWindow import MainWindow
from PyQt4.QtGui import *
import sys
__author__ = 'caja'


def main():
    app = QApplication(sys.argv)
    form = MainWindow()
    form.show()
    form.on_load()
    app.exec_()


if __name__ == '__main__':
    main()