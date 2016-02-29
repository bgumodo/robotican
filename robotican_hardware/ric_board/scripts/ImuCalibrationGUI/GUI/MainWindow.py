import shlex

__author__ = 'tom'
from PyQt4.QtGui import *
from GUI.Scheme.imuCalib import Ui_imuCalib
import rospy
from ric_board.srv._calibIMU import calibIMU, calibIMURequest
import subprocess


class MainWindow(QDialog, Ui_imuCalib):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self._isStartCalib = False


        self.calibButton.clicked.connect(self.startOrSaveCalib)
        self.refreshButton.clicked.connect(self.refresh)

        self.refresh()

    def refresh(self):
        try:
            rospy.wait_for_service('/imu_Calibration', timeout=0.2)
            self.status.setText("Connected")

        except rospy.ROSException:
            self.status.setText("Not connected")

    def startOrSaveCalib(self):
        try:
            if not self._isStartCalib:
                rospy.wait_for_service('/imu_Calibration', timeout=0.2)
                serviceCallBack = rospy.ServiceProxy('/imu_Calibration', calibIMU)

                request = calibIMURequest()
                request.status = calibIMURequest.START_CALIB
                request.xMax = float(self.maxXLineEdit.text())
                request.yMax = float(self.maxYLineEdit.text())
                request.zMax = float(self.maxZLineEdit.text())
                request.xMin = float(self.minXLineEdit.text())
                request.yMin = float(self.minYLineEdit.text())
                request.zMin = float(self.minZLineEdit.text())

                serviceCallBack(request)

                self.calibButton.setText("Save calibration")

                subprocess.Popen(shlex.split('rqt_plot /imu_calib_publisher/x'))
                subprocess.Popen(shlex.split('rqt_plot /imu_calib_publisher/y'))
                subprocess.Popen(shlex.split('rqt_plot /imu_calib_publisher/z'))

                self._isStartCalib = True
            else:
                rospy.wait_for_service('/imu_Calibration', timeout=0.2)
                serviceCallBack = rospy.ServiceProxy('/imu_Calibration', calibIMU)

                request = calibIMURequest()
                request.status = calibIMURequest.STOP_CALIB

                serviceCallBack(request)

                self.calibButton.setText("Start calibration")

                self._isStartCalib = False
        except rospy.ROSException:
            self.status.setText("Not connected")
        except:
            QMessageBox.critical(self, "Error", "All fields must be float type.")