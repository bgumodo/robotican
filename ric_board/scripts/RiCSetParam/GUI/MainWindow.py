__author__ = 'tom1231'
from PyQt4.QtGui import *
from GUI.Scheme.SetParams import Ui_main
import rospy.exceptions
from ric_board.srv._get_devs import get_devs, get_devsRequest
from ric_board.srv._setParam import setParam, setParamRequest


class MainWindow(QDialog, Ui_main):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self._devs = []

        self.refresh()

        self.refreshB.clicked.connect(self.refresh)
        self.setB.clicked.connect(self.sendNewParams)
        self.devList.itemSelectionChanged.connect(self.listChange)

        self._selected = False

    def refresh(self):
        try:
            rospy.wait_for_service('/devsOnline', timeout=0.2)

            self.status.setText('Connected')

            getDevsOnline = rospy.ServiceProxy('/devsOnline', get_devs)
            request = get_devsRequest()
            request.req = True
            responds = getDevsOnline(request)
            self._devs = responds.devs.devs

        except rospy.ROSException:
            self.status.setText('Not connected')
        finally:
            self.buildList()

    def buildList(self):
        self.devList.clear()
        self.removeAllFields()

        for device in self._devs:
            self.devList.addItem(QListWidgetItem(device.devName))

    def removeAllFields(self):
        for i in xrange(self.devFrame.layout().count()):
            self.devFrame.layout().itemAt(i).widget().deleteLater()

    def sendNewParams(self):
        try:
            if not self._selected: return
            rospy.wait_for_service('/devsSetParam', timeout=0.2)
            motor = self._devs[self.devList.currentRow()]

            kp = float(self.devFrame.layout().itemAt(1).widget().text())
            ki = float(self.devFrame.layout().itemAt(3).widget().text())
            kd = float(self.devFrame.layout().itemAt(5).widget().text())

            motor.values = [kp, ki, kd]

            request = setParamRequest()

            request.dev.devName = motor.devName
            request.dev.type = motor.type
            request.dev.values = [kp, ki, kd]

            setNewParam = rospy.ServiceProxy('/devsSetParam', setParam)

            setNewParam(request)

        except rospy.ROSException:
            self.status.setText('Not connected')

    def listChange(self):
        motor = self._devs[self.devList.currentRow()]
        self.removeAllFields()

        self.devFrame.layout().addRow(QLabel('Kp: '), QLineEdit(str(motor.values[0])))
        self.devFrame.layout().addRow(QLabel('Ki: '), QLineEdit(str(motor.values[1])))
        self.devFrame.layout().addRow(QLabel('Kd: '), QLineEdit(str(motor.values[2])))

        self._selected = True
