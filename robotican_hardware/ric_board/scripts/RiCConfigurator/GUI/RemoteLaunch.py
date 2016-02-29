import rospkg

__author__ = 'tom1231'
from PyQt4.QtGui import *
from GUI.Schemes.RemoteLaunch import Ui_Dialog
from os.path import basename
import os
import shlex
import subprocess

class RemoteLaunch(QDialog, Ui_Dialog):
    def __init__(self, parent=None):
        super(RemoteLaunch, self).__init__(parent)
        self.setupUi(self)
        self._file = ''

        self.launchButton.clicked.connect(self.launchRemote)
        self.browse.clicked.connect(self.browseLaunch)
        self.TestButton.clicked.connect(self.ping)
        self.newTerm.clicked.connect(self.newTermLaunch)

        self.isLaunch = False

    def launchRemote(self):
        if self._file is None or self._file == '' or str(self.hostIpAdd.text()) == '' or str(self.hostUser.text()) == '' or str(self.hostPassword.text()) == '' or str(self.localIP.text()) == '' or str(self._file) == '':
            QMessageBox.critical(self, "Error", "Please fill all the fields to launch a from remote.")
            return
        subprocess.Popen(shlex.split('gnome-terminal -e "rosrun ric_base_station remote_robot.sh %s %s %s %s %s"' % (str(self.hostIpAdd.text()),str(self.localIP.text()), str(self.hostUser.text()), str(self.hostPassword.text()), str(self._file))))
        print "rosrun ric_base_station remote_robot.sh %s %s %s %s %s" % (str(self.hostIpAdd.text()),str(self.localIP.text()), str(self.hostUser.text()), str(self.hostPassword.text()), str(self._file))
        self.isLaunch = True

    def newTermLaunch(self):
        if not self.isLaunch:
            QMessageBox.critical(self, "Error", "Please start the launch first.")
            return

        pkg = rospkg.RosPack().get_path('ric_board')
        subprocess.Popen(shlex.split('gnome-terminal -e "%s/scripts/envGui.sh %s %s"' % (pkg, str(self.hostIpAdd.text()), str(self.localIP.text()))))

    def browseLaunch(self):
        self._file = basename(str(QFileDialog.getOpenFileName(self, self.tr("Choose remote launch"), '.', self.tr("Launch file (*.launch)"))))
        if self._file is None or self._file == '': return
        self.path.setText(self._file)

    def ping(self):
        if str(self.hostIpAdd.text()) == '':
            QMessageBox.critical(self, "Error", "Please fill robot ip first.")
            return
        code = os.system('ping -c 1 %s' % (self.hostIpAdd.text()))
        if code > 0:
            QMessageBox.information(self, "Ping info", "Robot is unreachable")
            return
        QMessageBox.information(self, "Ping info", "Robot is reachable")


