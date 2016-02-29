from BAL.Devices.KeyboardTeleop import KeyboardTeleop
from BAL.Devices.emergencySwitch import EmergencySwitch
from BAL.Devices.joystickTeleop import JoystickTeleop
from BAL.Devices.launchFile import RosLaunch
from BAL.Devices.rosNode import RosNode
from BAL.Devices.velocitySmoother import VelocitySmoother
from GUI.AboutWindow import About
from GUI.SimulationWindow import SimulationWindow

__author__ = 'tom1231'
import rospkg
import shlex
from xml.etree import ElementTree
from lxml.etree import Element, SubElement
from xml.dom import minidom
from BAL.Devices.CloseLoopTwo import CloseLoopTwo
from BAL.Devices.Battery import Battery
from BAL.Devices.CloseLoop import CloseLoop
from BAL.Devices.DiffClose import DiffClose
from BAL.Devices.DiffCloseFour import DiffCloseFour
from BAL.Devices.DiffOpen import DiffOpen
from BAL.Devices.Gps import Gps
from BAL.Devices.Hokuyo import Hokuyo
from BAL.Devices.Imu import Imu
from BAL.Devices.OpenLoop import OpenLoop
from BAL.Devices.Openni import Opennni
from BAL.Devices.PPMReader import PPMReader
from BAL.Devices.Ppm import Ppm
from BAL.Devices.Relay import Relay
from BAL.Devices.RobotModel import RobotModel
from BAL.Devices.Servo import Servo
from BAL.Devices.Slam import Slam
from BAL.Devices.Switch import Switch
from BAL.Devices.Urf import Urf
from BAL.Devices.UsbCam import UsbCam
from BAL.Interface.DeviceFrame import SERVO, BATTERY, SWITCH, IMU, PPM, GPS, RELAY, URF, CLOSE_LOP_ONE, CLOSE_LOP_TWO, \
    OPEN_LOP, DIFF_CLOSE, DIFF_OPEN, EX_DEV, HOKUYO, OPRNNI, USBCAM, DIFF_CLOSE_FOUR, ROBOT_MODEL, SLAM, Keyboard, \
    JOYSTICK, SMOOTHER, LAUNCH, NODE, EMERGENCY_SWITCH
from GUI.RemoteLaunch import RemoteLaunch
from GUI.ShowRiCBoard import ShowRiCBoard

from PyQt4.QtGui import *
from Schemes.main import Ui_MainWindow
import webbrowser
import pickle
from os import system, path
import subprocess
import re


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.data = []
        self.motors = []
        self.currentShowDev = None
        self.root = Element('launch')

        self.actionAbout_RIC.triggered.connect(self.about)
        self.actionServo.triggered.connect(self.addServo)
        self.actionBattery_Monitor.triggered.connect(self.addBattery)
        self.actionSwitch.triggered.connect(self.addSwitch)
        self.actionIMU.triggered.connect(self.addImu)
        self.actionPPM.triggered.connect(self.addPpm)
        self.actionGPS.triggered.connect(self.addGps)
        self.actionRelay.triggered.connect(self.addRelay)
        self.actionURF.triggered.connect(self.addURF)
        self.actionMotor_with_one_encoder.triggered.connect(self.addCloseMotorOne)
        self.actionMotor_with_two_encoder.triggered.connect(self.addCloseMotorTwo)
        self.actionOpen_Loop.triggered.connect(self.addOpenMotor)
        self.actionWith_two_motors.triggered.connect(self.addDiffClose)
        self.actionWith_four_motors.triggered.connect(self.addDiffCloseFour)
        self.actionOpen_Loop_Drive.triggered.connect(self.addDiffOpen)
        self.actionUSB_Camera.triggered.connect(self.addUsbCam)
        self.actionOPENNI.triggered.connect(self.addOpenni)
        self.actionHakoyo.triggered.connect(self.addOHokuyo)
        self.actionSave.triggered.connect(self.save)
        self.actionOpen.triggered.connect(self.load)
        self.actionNew.triggered.connect(self.new)
        self.actionReconfig_RiC_Board.triggered.connect(self.configRiCBoard)
        self.actionRobot_Model.triggered.connect(self.addRobotModel)
        self.actionAbout_RiC_Board.triggered.connect(self.aboutRiCBoard)
        self.actionSLAM.triggered.connect(self.addSLAM)
        self.actionRemote_robot_launch.triggered.connect(self.launchRemote)
        self.actionPPM_Reader.triggered.connect(self.addPPmReader)
        self.actionSet_parameters.triggered.connect(self.paramManager)
        self.actionKeyboard.triggered.connect(self.addKeyboard)
        self.actionJoystick.triggered.connect(self.addJoystick)
        self.actionDifferential_Drive_smoother.triggered.connect(self.addDiffSmooth)
        self.actionAbout.triggered.connect(self.showAbout)
        self.actionImu_calibration.triggered.connect(self.showImuCalib)
        #self.actionRobot_simulation.triggered.connect(self.startSimGUI)
        self.actionInclude_ros_launch.triggered.connect(self.addRosLaunch)
        self.actionInclude_ros_node.triggered.connect(self.addRosNode)
        self.actionEmegency_switch.triggered.connect(self.addEmergencySwitch)

        self.fileName.textChanged.connect(self.fileNameEven)
        self.nameSpace.textChanged.connect(self.namespaceEven)

        self.devList.itemPressed.connect(self.clickListEven)
        self.devList.doubleClicked.connect(self.devEdit)

        self.servoPorts = QComboBox()
        self.servoPorts.addItems([
            self.tr('1'),
            self.tr('2')
        ])

        self.switchPorts = QComboBox()
        self.switchPorts.addItems([
            self.tr('1'),
            self.tr('2')
        ])

        self.relayPorts = QComboBox()
        self.relayPorts.addItems([
            self.tr('1'),
            self.tr('2')
        ])

        self.urfPorts = QComboBox()
        self.urfPorts.addItems([
            self.tr('1'),
            self.tr('2'),
            self.tr('3')
        ])

        self.encoders = QComboBox()
        self.encoders.addItems([
            self.tr('1'),
            self.tr('2'),
            self.tr('3'),
            self.tr('4')
        ])

        self._ns = ''
        self._fileName = ''

        self.haveBattery = False
        self.haveIMU = False
        self.havePPM = False
        self.haveGPS = False
        self.haveCloseLoop = False
        self.haveOpenLoop = False
        self.haveDiff = False
        self.haveReader = False


        self.diffEnable = False

        self.editMode = False
        self.listMode = False
        self.newDevMode = False
        self.override = True
        self.pushButton_2.setEnabled(False)
        self.pushButton_2.clicked.connect(self.launch)

        allDev = subprocess.check_output(shlex.split("ls /dev"))

        conDevs = re.findall('ttyUSB.*', allDev) + re.findall('ttyACM.*', allDev) + re.findall('RiCBoard', allDev)

        for dev in conDevs: self.ConPortList.addItem(self.tr(dev))

        self.ConPortList.setCurrentIndex(self.ConPortList.count() - 1)

    def startSimGUI(self):
        pass
        # dialog = SimulationWindow()
        # dialog.show()
        # dialog.exec_()

    def showAbout(self):
        dialog = About(self)
        dialog.show()
        dialog.exec_()

    def launchRemote(self):
        dialog = RemoteLaunch(self)
        dialog.show()
        dialog.exec_()

    def paramManager(self):
        subprocess.Popen(shlex.split('roslaunch ric_board startParamMsg.launch'))

    def showImuCalib(self):
        subprocess.Popen(shlex.split('roslaunch ric_board startImuCalib.launch'))

    def about(self):
        webbrowser.open('http://wiki.ros.org/ric_board?distro=indigo')

    def aboutRiCBoard(self):
        dialog = ShowRiCBoard(self)
        dialog.show()
        dialog.exec_()

    def configRiCBoard(self):
        pkg = rospkg.RosPack().get_path('ric_board')
        path = QFileDialog.getOpenFileName(self, self.tr("Load File"), "%s/setup" % pkg, self.tr("Hex files (*.hex)"))
        exitStatus = system("%s/setup/board_loader --mcu=mk20dx256 -sv %s" % (pkg, path))
        if exitStatus > 0:
            QMessageBox.critical(self, "Error", "Could not build RiCBoard.")
        else:
            QMessageBox.information(self, "Done", "Firmware successfully updated.")

    def new(self):
        self.interruptHandler()
        size = self.devList.count()
        for i in xrange(size):
            self.devList.takeItem(0)
        self.data = []
        self.motors = []
        self.currentShowDev = None
        self.root = Element('launch')

        self.servoPorts = QComboBox()
        self.servoPorts.addItems([
            self.tr('1'),
            self.tr('2')
        ])

        self.switchPorts = QComboBox()
        self.switchPorts.addItems([
            self.tr('1'),
            self.tr('2')
        ])

        self.relayPorts = QComboBox()
        self.relayPorts.addItems([
            self.tr('1'),
            self.tr('2')
        ])

        self.urfPorts = QComboBox()
        self.urfPorts.addItems([
            self.tr('1'),
            self.tr('2'),
            self.tr('3')
        ])

        self.encoders = QComboBox()
        self.encoders.addItems([
            self.tr('1'),
            self.tr('2'),
            self.tr('3'),
            self.tr('4')
        ])

        self._ns = ''
        self._fileName = ''

        self.haveBattery = False
        self.haveIMU = False
        self.havePPM = False
        self.haveGPS = False
        self.haveCloseLoop = False
        self.haveOpenLoop = False
        self.haveDiff = False
        self.haveReader = False


        self.diffEnable = False

        self.editMode = False
        self.listMode = False
        self.newDevMode = False
        self.override = True
        self.pushButton_2.setEnabled(False)

        self.fileName.setText(self._fileName)
        self.nameSpace.setText(self._ns)

    def launch(self):
        pkg = rospkg.RosPack().get_path('ric_board')
        if path.isfile('%s/DATA/%s.RIC' % (pkg, self._fileName)):
            devices = pickle.load(open('%s/DATA/%s.RIC' % (pkg, self._fileName)))[2]
            newDevices = []

            for dev in self.data:
                newDevices.append(dev.toDict())

            if devices != newDevices:
                ans = QMessageBox.warning(self, "Warning",
                                          "There is some changes in the file, if you don`t save them the 'RiCboard' won`t be able to recognize the changes.\n\nDo you want to save before launch?",
                                          QMessageBox.Yes | QMessageBox.No)
                if ans == QMessageBox.Yes:
                    self.override = True
                    self.save()
        subprocess.Popen(shlex.split("gnome-terminal --command='roslaunch ric_board %s.launch'" % self._fileName))

    def load(self):
        pkg = rospkg.RosPack().get_path('ric_board')
        fileName = QFileDialog.getOpenFileName(self, self.tr("Load File"), "%s/DATA" % pkg, self.tr("RiC File (*.RIC)"))
        if fileName != '':
            self.new()
            self.override = False
            load = open(fileName)
            data = pickle.load(load)

            self._fileName = data[0]
            self._ns = data[1]

            yaml = open("%s/config/%s.yaml" % (pkg, self._fileName))

            conDev = yaml.readline().split(': ')[1][:-1]

            allDevs = [str(self.ConPortList.itemText(i)) for i in xrange(self.ConPortList.count())]

            for i in xrange(len(allDevs)):
                if conDev == allDevs[i]:
                    self.ConPortList.setCurrentIndex(i)
                    break

            self.nameSpace.setText(self._ns)
            self.fileName.setText(self._fileName)

            devices = data[2]

            # print devices

            for dev in devices:
                if dev['type'] == BATTERY:
                    self.currentShowDev = Battery(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == SERVO:
                    self.currentShowDev = Servo(self.DevFrame, self.data, self.servoPorts)
                    self.currentShowDev.fromDict(dev)
                    self.servoPorts.removeItem(self.currentShowDev.findItem())
                elif dev['type'] == SWITCH:
                    self.currentShowDev = Switch(self.DevFrame, self.data, self.switchPorts)
                    self.currentShowDev.fromDict(dev)
                    self.switchPorts.removeItem(self.currentShowDev.findItem())
                elif dev['type'] == IMU:
                    self.currentShowDev = Imu(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == PPM:
                    self.currentShowDev = Ppm(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == GPS:
                    self.currentShowDev = Gps(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == RELAY:
                    self.currentShowDev = Relay(self.DevFrame, self.data, self.relayPorts)
                    self.currentShowDev.fromDict(dev)
                    self.relayPorts.removeItem(self.currentShowDev.findItem())
                elif dev['type'] == URF:
                    self.currentShowDev = Urf(self.DevFrame, self.data, self.urfPorts)
                    self.currentShowDev.fromDict(dev)
                    self.urfPorts.removeItem(self.currentShowDev.findItem())
                elif dev['type'] == CLOSE_LOP_ONE:
                    self.currentShowDev = CloseLoop(self.DevFrame, self.data, self.encoders)
                    self.currentShowDev.fromDict(dev)
                    self.encoders.removeItem(self.currentShowDev.findItem())
                elif dev['type'] == CLOSE_LOP_TWO:
                    self.currentShowDev = CloseLoopTwo(self.DevFrame, self.data, self.encoders)
                    self.currentShowDev.fromDict(dev)
                    self.encoders.removeItem(self.currentShowDev.findItem())
                    self.encoders.removeItem(self.currentShowDev.findItem2())
                elif dev['type'] == OPEN_LOP:
                    self.currentShowDev = OpenLoop(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == DIFF_CLOSE:
                    self.currentShowDev = DiffClose(self.DevFrame, self.data, self.motors)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == DIFF_CLOSE_FOUR:
                    self.currentShowDev = DiffCloseFour(self.DevFrame, self.data, self.motors)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == DIFF_OPEN:
                    self.currentShowDev = DiffOpen(self.DevFrame, self.data, self.motors)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == HOKUYO:
                    self.currentShowDev = Hokuyo(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == OPRNNI:
                    self.currentShowDev = Opennni(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == USBCAM:
                    self.currentShowDev = UsbCam(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == ROBOT_MODEL:
                    self.currentShowDev = RobotModel(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == SLAM:
                    self.currentShowDev = Slam(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == PPMReader:
                    self.currentShowDev = PPMReader(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == Keyboard:
                    self.currentShowDev = KeyboardTeleop(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == JOYSTICK:
                    self.currentShowDev = JoystickTeleop(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == SMOOTHER:
                    self.currentShowDev = VelocitySmoother(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == LAUNCH:
                    self.currentShowDev = RosLaunch(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == NODE:
                    self.currentShowDev = RosNode(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)
                elif dev['type'] == EMERGENCY_SWITCH:
                    self.currentShowDev = EmergencySwitch(self.DevFrame, self.data)
                    self.currentShowDev.fromDict(dev)

                if self.currentShowDev.getDevType() == BATTERY:
                    self.haveBattery = True
                if self.currentShowDev.getDevType() == IMU:
                    self.haveIMU = True
                if self.currentShowDev.getDevType() == PPM:
                    self.havePPM = True
                if self.currentShowDev.getDevType() == GPS:
                    self.haveGPS = True



                if (self.currentShowDev.getDevType() == CLOSE_LOP_ONE) or (
                            self.currentShowDev.getDevType() == CLOSE_LOP_TWO):
                    self.haveCloseLoop = True
                    self.motors.append(self.currentShowDev.getName())
                if self.currentShowDev.getDevType() == OPEN_LOP:
                    self.haveOpenLoop = True
                    self.motors.append(self.currentShowDev.getName())
                if self.currentShowDev.getDevType() == DIFF_CLOSE or self.currentShowDev.getDevType() == DIFF_OPEN or self.currentShowDev.getDevType() == DIFF_CLOSE_FOUR:
                    self.haveDiff = True
                    self.diffEnable = True
                self.devList.addItem(QListWidgetItem(self.currentShowDev.getName()))
                self.data.append(self.currentShowDev)
                self.currentShowDev = None
            self.pushButton_2.setEnabled(True)

    def save(self):
        pkg = rospkg.RosPack().get_path('ric_board')
        if len(self.data) == 0:
            QMessageBox.critical(self, "File error", "Can not save a empty file.")
            return
        if self._fileName == '':
            QMessageBox.critical(self, "File error", "Can not save file without a name.")
            return
        if not self.override and path.isfile('%s/config/%s.yaml' % (pkg, self._fileName)):
            ans = QMessageBox.question(self, "Override", "Do you want to override this file",
                                       QMessageBox.Yes | QMessageBox.No)
            if ans == QMessageBox.Yes:
                self.override = True
            else:
                return

        parent = self.root
        if self._ns != '':
            parent = SubElement(self.root, 'group', {'ns': self._ns})
        for dev in self.data:
            if dev.getDevType() != EX_DEV and dev.isToSave():
                at = {
                    'pkg': 'ric_board',
                    'type': 'Start.py',
                    'name': 'RiCTraffic',
                    'output': 'screen'
                }

                SubElement(parent, 'node', at)
                break
        initDiffClose = '0'
        initDiffOpen = '0'
        initDiffCloseFour = '0'
        initIMU = '0'
        initGPS = '0'
        initPPM = '0'
        initBAT = '0'
        toSave = open("%s/config/%s.yaml" % (pkg, self._fileName), 'w')
        launch = open("%s/launch/%s.launch" % (pkg, self._fileName), 'w')
        if str(self.ConPortList.currentText()) != '':
            toSave.write("CON_PORT: %s\n" % str(self.ConPortList.currentText()))
        else:
            toSave.write("CON_PORT: RiCBoard\n")
        toSave.write("FILE_NAME: %s\n" % self._fileName)
        for dev in self.data:
            if dev.isToSave():
                if dev.getDevType() == EX_DEV:
                    # if dev.toDict()['type'] == ROBOT_MODEL: dev.saveToFile(self.root)
                    # else: dev.saveToFile(parent)
                    dev.saveToFile(parent)
                else:
                    dev.saveToFile(toSave)

                    if dev.getDevType() == DIFF_OPEN:
                        initDiffOpen = '1'
                    elif dev.getDevType() == DIFF_CLOSE:
                        initDiffClose = '1'
                    elif dev.getDevType() == DIFF_CLOSE_FOUR:
                        initDiffCloseFour = '1'
                    elif dev.getDevType() == IMU:
                        initIMU = '1'
                    elif dev.getDevType() == GPS:
                        initGPS = '1'
                    elif dev.getDevType() == PPM:
                        initPPM = '1'
                    elif dev.getDevType() == BATTERY:
                        initBAT = '1'

        toSave.write('IMU_INIT: ' + initIMU + '\n')
        toSave.write('GPS_INIT: ' + initGPS + '\n')
        toSave.write('PPM_INIT: ' + initPPM + '\n')
        toSave.write('BAT_INIT: ' + initBAT + '\n')
        toSave.write('DIFF_INIT: ' + initDiffClose + '\n')
        toSave.write('DIFF_INIT_OP: ' + initDiffOpen + '\n')
        toSave.write('DIFF_CLOSE_FOUR: ' + initDiffCloseFour + '\n')

        toSave.write('closeLoopNum: ' + str(CloseLoop.closeLoop) + '\n')
        toSave.write('switchNum: ' + str(Switch.switchCount) + '\n')
        toSave.write('servoNum: ' + str(Servo.servoCount) + '\n')
        toSave.write('relayNum: ' + str(Relay.relayCount) + '\n')
        toSave.write('URFNum: ' + str(Urf.urfCount) + '\n')
        toSave.write('openLoopNum: ' + str(OpenLoop.openLoopNum) + '\n')
        toSave.write('emergencySwitchNum: ' + str(EmergencySwitch.emergency_switch_count) + '\n')

        SubElement(parent, 'rosparam', {
            'file': '$(find ric_board)/config/' + self._fileName + '.yaml',
            'command': 'load'
        })
        launch.write(prettify(self.root))

        fileData = open('%s/DATA/%s.RIC' % (pkg, self._fileName), 'wb')

        ls = []
        for dev in self.data:
            ls.append(dev.toDict())

        pickle.dump([self._fileName, self._ns, ls], fileData)

        toSave.close()
        launch.close()
        self.root = Element('launch')
        Servo.servoCount = 0
        Relay.relayCount = 0
        Urf.urfCount = 0
        Switch.switchCount = 0
        CloseLoop.closeLoop = 0
        OpenLoop.openLoopNum = 0
        # QMessageBox.information(self, 'File', 'File saved')

        QMessageBox.information(self, "File Saved", "To launch: $ roslaunch ric_board %s.launch" % self._fileName)
        self.pushButton_2.setEnabled(True)

    def addEmergencySwitch(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = EmergencySwitch(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addRosNode(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = RosNode(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addRosLaunch(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = RosLaunch(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addDiffSmooth(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = VelocitySmoother(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addJoystick(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = JoystickTeleop(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addKeyboard(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = KeyboardTeleop(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addPPmReader(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = PPMReader(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addSLAM(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Slam(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addRobotModel(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = RobotModel(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addDiffCloseFour(self):
        if not self.haveCloseLoop or len(self.motors) < 4:
            QMessageBox.critical(self, "Driver error", "Need to have at less four close loop motors.")
            return
        if self.haveDiff:
            QMessageBox.critical(self, "Driver error", "Can not have more.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = DiffCloseFour(self.DevFrame, self.data, self.motors)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addOpenni(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Opennni(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addOHokuyo(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Hokuyo(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addUsbCam(self):
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = UsbCam(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addDiffOpen(self):
        if not self.haveOpenLoop or len(self.motors) < 2:
            QMessageBox.critical(self, "Driver error", "Need to have at less two open loop motors.")
            return
        if self.haveDiff:
            QMessageBox.critical(self, "Driver error", "Can not have more.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = DiffOpen(self.DevFrame, self.data, self.motors)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addDiffClose(self):
        if not self.haveCloseLoop or len(self.motors) < 2:
            QMessageBox.critical(self, "Driver error", "Need to have at less two close loop motors.")
            return
        if self.haveDiff:
            QMessageBox.critical(self, "Driver error", "Can not have more.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = DiffClose(self.DevFrame, self.data, self.motors)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addOpenMotor(self):
        if self.haveCloseLoop:
            QMessageBox.critical(self, "Error", "Open and close motors can not exist in the same configuration.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = OpenLoop(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addCloseMotorTwo(self):
        if self.haveOpenLoop:
            QMessageBox.critical(self, "Error", "Open and close motors can not exist in the same configuration.")
            return
        if self.encoders.count() < 2:
            QMessageBox.critical(self, "Close Motor Error", "Need two or more encoder ports to build this motor.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = CloseLoopTwo(self.DevFrame, self.data, self.encoders)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addCloseMotorOne(self):
        if self.haveOpenLoop:
            QMessageBox.critical(self, "Error", "Open and close motors can not exist in the same configuration.")
            return
        if self.encoders.count() == 0:
            QMessageBox.critical(self, "Close Motor Error", "Out of encoder ports.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = CloseLoop(self.DevFrame, self.data, self.encoders)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addURF(self):
        if self.urfPorts.count() == 0:
            QMessageBox.critical(self, "URF Error", "Out of URF ports.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Urf(self.DevFrame, self.data, self.urfPorts)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addBattery(self):
        if self.haveBattery:
            QMessageBox.critical(self, "Battery Error", "Can't add another battery to ric board.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Battery(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addServo(self):
        if self.servoPorts.count() == 0:
            QMessageBox.critical(self, "Servo Error", "Out of servo ports.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Servo(self.DevFrame, self.data, self.servoPorts)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addSwitch(self):
        if self.switchPorts.count() == 0:
            QMessageBox.critical(self, "Switch Error", "Out of switch ports.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Switch(self.DevFrame, self.data, self.switchPorts)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addImu(self):
        if self.haveIMU:
            QMessageBox.critical(self, "IMU Error", "Can't add another IMU to ric board.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Imu(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addPpm(self):
        if self.havePPM:
            QMessageBox.critical(self, "PPM Error", "Can't add another PPM to ric board.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Ppm(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addGps(self):
        if self.haveGPS:
            QMessageBox.critical(self, "GPS Error", "Can't add another GPS to ric board.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Gps(self.DevFrame, self.data)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def addRelay(self):
        if self.relayPorts.count() == 0:
            QMessageBox.critical(self, "Relay Error", "Out of relay ports.")
            return
        self.interruptHandler()
        self.newDevMode = True
        self.currentShowDev = Relay(self.DevFrame, self.data, self.relayPorts)
        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.addDevToList)

    def clickListEven(self):
        self.interruptHandler()
        index = self.devList.currentRow()
        self.listMode = True
        self.currentShowDev = self.data[index]
        self.currentShowDev.printDetails()

        self.Edit.clicked.connect(self.devEdit)
        self.Delete.clicked.connect(self.devDelete)
        self.saveStatus.clicked.connect(self.changeDevSaveStatus)

    def changeDevSaveStatus(self):
        self.interruptHandler()
        index = self.devList.currentRow()
        self.currentShowDev = self.data[index]

        if self.currentShowDev.getDevType() in [CLOSE_LOP_ONE, CLOSE_LOP_TWO, OPEN_LOP] and self.diffEnable:
            QMessageBox.critical(self, "Error", "Can't disable a motor while differential drive is present")
            return

        motorCount = 0
        if self.currentShowDev.getDevType() in [DIFF_CLOSE, DIFF_OPEN, DIFF_CLOSE_FOUR] and not self.diffEnable:
            for dev in self.data:
                if dev.getDevType() in [CLOSE_LOP_ONE, CLOSE_LOP_TWO, OPEN_LOP] and dev.isToSave(): motorCount += 1

            if (motorCount <= 1) or (motorCount < 4 and self.currentShowDev.getDevType() == CLOSE_LOP_ONE):
                QMessageBox.critical(self, "Error", "Can't enable  differential drive motor while motors are disable")
                return

        if self.currentShowDev.getDevType() in [DIFF_CLOSE, DIFF_OPEN, DIFF_CLOSE_FOUR]:
            self.diffEnable = not self.currentShowDev.isToSave()

        if self.currentShowDev.isToSave():
            self.devList.item(index).setForeground(QColor(255, 0, 0))
            self.currentShowDev.setToSave(False)
        else:
            self.devList.item(index).setForeground(QColor(0, 0, 0))
            self.currentShowDev.setToSave(True)

    def namespaceEven(self, text):
        self._ns = str(text)

    def fileNameEven(self, text):
        self._fileName = str(text)
        self.pushButton_2.setEnabled(False)

    def devDelete(self):
        if self.currentShowDev.getDevType() == SERVO:
            self.servoPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == BATTERY:
            self.haveBattery = False
        if self.currentShowDev.getDevType() == SWITCH:
            self.switchPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == IMU:
            self.haveIMU = False
        if self.currentShowDev.getDevType() == PPM:
            self.havePPM = False
        if self.currentShowDev.getDevType() == GPS:
            self.haveGPS = False
        if self.currentShowDev.getDevType() == RELAY:
            self.relayPorts.addItem(self.currentShowDev.getPort())

        if self.currentShowDev.getDevType() == URF:
            self.urfPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == CLOSE_LOP_ONE:
            if self.haveDiff:
                QMessageBox.critical(self, "Error", "Can't delete a motor while differential drive is present")
                return
            self.motors.remove(self.currentShowDev.getName())
            self.encoders.addItem(self.currentShowDev.getEncoder())
        if self.currentShowDev.getDevType() == CLOSE_LOP_TWO:
            if self.haveDiff:
                QMessageBox.critical(self, "Error", "Can't delete a motor while differential drive is present")
                return
            self.motors.remove(self.currentShowDev.getName())
            self.encoders.addItem(self.currentShowDev.getEncoders()[0])
            self.encoders.addItem(self.currentShowDev.getEncoders()[1])
        if self.currentShowDev.getDevType() == OPEN_LOP:
            if self.haveDiff:
                QMessageBox.critical(self, "Error", "Can't delete a motor while differential drive is present")
                return
            self.motors.remove(self.currentShowDev.getName())
        if self.currentShowDev.getDevType() == DIFF_CLOSE or self.currentShowDev.getDevType() == DIFF_OPEN or self.currentShowDev.getDevType() == DIFF_CLOSE_FOUR:
            self.haveDiff = False
            self.diffEnable = False

        self.data.remove(self.currentShowDev)
        self.devList.takeItem(self.devList.currentRow())
        self.removeAllFields()

        if len(self.motors) == 0 and self.haveCloseLoop: self.haveCloseLoop = False
        if len(self.motors) == 0 and self.haveOpenLoop: self.haveOpenLoop = False

        self.Edit.clicked.disconnect(self.devEdit)
        self.Delete.clicked.disconnect(self.devDelete)
        self.listMode = False

    def devEdit(self):
        self.interruptHandler()
        self.editMode = True
        if self.currentShowDev.getDevType() == SERVO:
            self.servoPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == SWITCH:
            self.switchPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == RELAY:
            self.relayPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == URF:
            self.urfPorts.addItem(self.currentShowDev.getPort())
        if self.currentShowDev.getDevType() == CLOSE_LOP_ONE:
            self.encoders.addItem(self.currentShowDev.getEncoder())
        if self.currentShowDev.getDevType() == CLOSE_LOP_TWO:
            self.encoders.addItem(self.currentShowDev.getEncoders()[0])
            self.encoders.addItem(self.currentShowDev.getEncoders()[1])

        self.currentShowDev.showDetails()
        self.pushButton.clicked.connect(self.editList)

    def removeAllFields(self):
        for i in xrange(self.DevFrame.layout().count()):
            self.DevFrame.layout().itemAt(i).widget().deleteLater()

    def interruptHandler(self):
        self.removeAllFields()
        if self.listMode:
            self.Edit.clicked.disconnect(self.devEdit)
            self.Delete.clicked.disconnect(self.devDelete)
            self.saveStatus.clicked.disconnect(self.changeDevSaveStatus)
            self.listMode = False
        if self.editMode:
            self.pushButton.clicked.disconnect(self.editList)
            if self.currentShowDev.getDevType() == SERVO:
                self.servoPorts.removeItem(self.currentShowDev.findItem())
            if self.currentShowDev.getDevType() == SWITCH:
                self.switchPorts.removeItem(self.currentShowDev.findItem())
            if self.currentShowDev.getDevType() == RELAY:
                self.relayPorts.removeItem(self.currentShowDev.findItem())
            if self.currentShowDev.getDevType() == URF:
                self.urfPorts.removeItem(self.currentShowDev.findItem())
            if self.currentShowDev.getDevType() == CLOSE_LOP_ONE:
                self.encoders.removeItem(self.currentShowDev.findItem())
            if self.currentShowDev.getDevType() == CLOSE_LOP_TWO:
                self.encoders.removeItem(self.currentShowDev.findItem())
                self.encoders.removeItem(self.currentShowDev.findItem2())
            self.editMode = False
        if self.newDevMode:
            self.pushButton.clicked.disconnect(self.addDevToList)
            self.newDevMode = False

    def removeFields(self):
        if self.currentShowDev.isValid():
            self.removeAllFields()
            self.pushButton.clicked.disconnect(self.removeFields)
            self.newDevMode = False

    def addDevToList(self):
        self.currentShowDev.add()

        if self.currentShowDev.isValid():
            if self.currentShowDev.getDevType() == BATTERY:
                self.haveBattery = True
            if self.currentShowDev.getDevType() == IMU:
                self.haveIMU = True
            if self.currentShowDev.getDevType() == PPM:
                self.havePPM = True
            if self.currentShowDev.getDevType() == GPS:
                self.haveGPS = True
            if (self.currentShowDev.getDevType() == CLOSE_LOP_ONE) or (
                        self.currentShowDev.getDevType() == CLOSE_LOP_TWO):
                self.haveCloseLoop = True
                self.motors.append(self.currentShowDev.getName())
            if self.currentShowDev.getDevType() == OPEN_LOP:
                self.haveOpenLoop = True
                self.motors.append(self.currentShowDev.getName())
            if self.currentShowDev.getDevType() == DIFF_CLOSE or self.currentShowDev.getDevType() == DIFF_OPEN or self.currentShowDev.getDevType() == DIFF_CLOSE_FOUR:
                self.haveDiff = True
                self.diffEnable = True

            self.devList.addItem(QListWidgetItem(self.currentShowDev.getName()))
            self.data.append(self.currentShowDev)
            self.removeAllFields()
            self.currentShowDev = None
            self.pushButton.clicked.disconnect(self.addDevToList)
            self.newDevMode = False

    def editList(self):
        oldName = self.currentShowDev.getName()
        self.currentShowDev.add()
        if self.currentShowDev.isValid():
            self.devList.currentItem().setText(self.currentShowDev.getName())
            self.data[self.devList.currentRow()] = self.currentShowDev
            if self.currentShowDev.getDevType() == OPEN_LOP or self.currentShowDev.getDevType() == CLOSE_LOP_ONE or self.currentShowDev.getDevType() == CLOSE_LOP_TWO:
                for i in xrange(len(self.motors)):
                    if self.motors[i] == oldName:
                        self.motors[i] = self.currentShowDev.getName()
                        break
            self.removeAllFields()
            self.currentShowDev = None
            self.pushButton.clicked.disconnect(self.editList)
            self.editMode = False
