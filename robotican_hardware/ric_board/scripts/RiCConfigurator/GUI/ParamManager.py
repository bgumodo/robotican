from _socket import SHUT_WR
import rospkg
import pickle
from BAL.Interface.DeviceFrame import CLOSE_LOP_ONE, CLOSE_LOP_TWO

__author__ = 'tom1231'
from PyQt4.QtGui import *
from GUI.Schemes.SetParams import Ui_main
from socket import socket, AF_INET, SOCK_STREAM, gethostname, error

BUF_SIZE = 1024
PORT = 1900

class ParamManager(QDialog, Ui_main):

    def __init__(self, parent=None):
        super(ParamManager, self).__init__(parent)
        self.setupUi(self)
        self.status.setText('connected')
        self._client = socket(AF_INET, SOCK_STREAM)
        self._client.connect((gethostname(), PORT))
        self._motors = []
        self.devList.itemSelectionChanged.connect(self.listChange)
        self._selected = False

        fileName = self._client.recv(BUF_SIZE)
        pkg = rospkg.RosPack().get_path('ric_board')
        dump = open("%s/DATA/%s.RIC" % (pkg, fileName))
        devs = pickle.load(dump)[2]
        for dev in devs:
            if (dev['type'] == CLOSE_LOP_ONE) or (dev['type'] == CLOSE_LOP_TWO):
                self._motors.append(dev)
                self.devList.addItem(QListWidgetItem(dev['name']))

        self.setB.clicked.connect(self.sendParam)
        self.finished.connect(self.closeWin)

    def listChange(self):
        motor = self._motors[self.devList.currentRow()]
        self.removeAllFields()

        self.devFrame.layout().addRow(QLabel('Kp: '), QLineEdit(motor['kp']))
        self.devFrame.layout().addRow(QLabel('Ki: '), QLineEdit(motor['ki']))
        self.devFrame.layout().addRow(QLabel('Kd: '), QLineEdit(motor['kd']))

        self._selected = True

    def closeWin(self):
        try:
            self._client.send('dis')
            if self._client.recv(BUF_SIZE) == 'by':
                    self._client.shutdown(SHUT_WR)
                    self._client.close()
        except error: pass

    def removeAllFields(self):
        for i in xrange(self.devFrame.layout().count()):
            self.devFrame.layout().itemAt(i).widget().deleteLater()

    def sendParam(self):
        try:
            if not self._selected: return
            motorNum = self.devList.currentRow()
            motor = self._motors[motorNum]

            kp = str(self.devFrame.layout().itemAt(1).widget().text())
            ki = str(self.devFrame.layout().itemAt(3).widget().text())
            kd = str(self.devFrame.layout().itemAt(5).widget().text())

            if kp != motor['kp']:
                self._client.send("motor|%d|1|%s" % (motorNum, kp))
            if ki != motor['ki']:
                self._client.send("motor|%d|2|%s" % (motorNum, ki))
            if kd != motor['kd']:
                self._client.send("motor|%d|3|%s" % (motorNum, kd))
        except error:
            QMessageBox.critical(self,'Error', 'RiCBoard not connected')
