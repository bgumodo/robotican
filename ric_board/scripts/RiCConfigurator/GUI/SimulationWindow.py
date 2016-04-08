import GUI.MainWindow

__author__ = 'tom'
from PyQt4.QtGui import *
from GUI.Schemes.gazeboGui import Ui_gazebo_gui
from BAL.Interface.DeviceFrame import SERVO, BATTERY, SWITCH, IMU, PPM, GPS, RELAY, URF, CLOSE_LOP_ONE, CLOSE_LOP_TWO, \
    OPEN_LOP, DIFF_CLOSE, DIFF_OPEN, EX_DEV, HOKUYO, OPRNNI, USBCAM, DIFF_CLOSE_FOUR, ROBOT_MODEL, SLAM, Keyboard, \
    JOYSTICK, SMOOTHER
import rospkg
import pickle
from PyQt4.QtCore import Qt
from lxml.etree import Element, SubElement


class SimulationWindow(QDialog, Ui_gazebo_gui):
    def __init__(self, parent=None):
        super(SimulationWindow, self).__init__(parent)
        self.setupUi(self)
        self._devs = []

        self.loadButton.clicked.connect(self.loadEvent)
        self.launchButton.clicked.connect(self.launchEvent)
        self.devList.itemClicked.connect(self.listChangeEvent)

        self.loadFile()

        self.showSimDetail()

    def listChangeEvent(self, item):
        dev = self._devs[self.devList.row(item)]
        if item.checkState() > 0:
            dev[1] = True
        else:
            dev[1] = False

    def loadFile(self):
        self._devs = []
        pkg = rospkg.RosPack().get_path('ric_board')
        fileName = QFileDialog.getOpenFileName(self, self.tr("Open file"), "%s/DATA" % pkg, self.tr("RiC File (*.RIC)"))
        if fileName == '': return

        devices = pickle.load(open(fileName))[2]
        self.arrangeDevices(devices)

    def arrangeDevices(self, devices):
        for dev in devices:
            if dev['type'] in [DIFF_CLOSE, IMU, OPRNNI, HOKUYO, USBCAM, URF]:
                self._devs.append([dev, True])

    def showSimDetail(self):
        for dev in self._devs:
            if dev[0]['type'] == OPRNNI:
                listItem = QListWidgetItem('OpenniCamera')
            else:
                listItem = QListWidgetItem(dev[0]['name'])

            listItem.setCheckState(Qt.Checked)
            self.devList.addItem(listItem)

    def clearLst(self):
        size = self.devList.count()
        for i in xrange(size):
            self.devList.takeItem(0)

    def loadEvent(self):
        self.loadFile()
        self.clearLst()
        self.showSimDetail()

    def launchEvent(self):
        root = Element('launch')

        SubElement(root, 'arg', {
            'name': 'paused',
            'default': 'false'
        })
        SubElement(root, 'arg', {
            'name': 'use_sim_time',
            'default': 'true'
        })
        SubElement(root, 'arg', {
            'name': 'gui',
            'default': 'true'
        })
        SubElement(root, 'arg', {
            'name': 'headless',
            'default': 'false'
        })
        SubElement(root, 'arg', {
            'name': 'debug',
            'default': 'false'
        })

        world = SubElement(root, 'include', dict(file='$(find gazebo_ros)/launch/empty_world.launch'))
        SubElement(world, 'arg', {
            'name': 'debug',
            'value': '$(arg debug)'
        })
        SubElement(world, 'arg', {
            'name': 'gui',
            'value': '$(arg gui)'
        })
        SubElement(world, 'arg', {
            'name': 'paused',
            'value': '$(arg paused)'
        })
        SubElement(world, 'arg', {
            'name': 'use_sim_time',
            'value': '$(arg use_sim_time)'
        })
        SubElement(world, 'arg', {
            'name': 'headless',
            'value': '$(arg headless)'
        })

        SubElement(root, 'param', {
            'name': 'robot_description',
            'command': "$(find xacro)/xacro.py '$(find ric_gazebo)/robots/komodo/komodo.xacro'  ns:='init' color_name:='Grey'"
        })

        haveCam = 'false'
        haveOpenNi = 'false'
        haveLaser = 'false'
        haveUrf = 'false'
        haveDiff = 'false'
        haveImu = 'false'

        for dev in self._devs:
            if dev[1]:
                if dev[0]['type'] == DIFF_CLOSE: haveDiff = 'true'
                if dev[0]['type'] == IMU: haveImu = 'true'
                if dev[0]['type'] == OPRNNI: haveOpenNi = 'true'
                if dev[0]['type'] == HOKUYO: haveLaser = 'true'
                if dev[0]['type'] == USBCAM: haveCam = 'true'
                if dev[0]['type'] == URF: haveUrf = 'true'

        amount = self.numberOfRobotsSpinBox.value()
        for i in xrange(amount):
            robotFile = SubElement(root, 'include', {'file': '$(find ric_gazebo)/launch/spawn_komodo.launch'})
            SubElement(robotFile, 'arg', dict(name='name', value='komodo_%d' % (i + 1)))
            SubElement(robotFile, 'arg', dict(name='color', value='White'))
            SubElement(robotFile, 'arg', dict(name='x', value='0.0'))
            SubElement(robotFile, 'arg', dict(name='y', value='%d.0' % i))
            SubElement(robotFile, 'arg', dict(name='z', value='0.1'))
            SubElement(robotFile, 'arg', dict(name='R', value='0.0'))
            SubElement(robotFile, 'arg', dict(name='P', value='0.0'))
            SubElement(robotFile, 'arg', dict(name='Y', value='0.0'))
            SubElement(robotFile, 'arg', dict(name='arm_camera', value='true'))

            SubElement(robotFile, 'arg', dict(name='front_camera', value=haveCam))
            SubElement(robotFile, 'arg', dict(name='isDiff', value=haveDiff))
            SubElement(robotFile, 'arg', dict(name='depth_camera', value=haveOpenNi))
            SubElement(robotFile, 'arg', dict(name='laser_scanner', value=haveLaser))
            SubElement(robotFile, 'arg', dict(name='urf', value=haveUrf))
            SubElement(robotFile, 'arg', dict(name='imu', value=haveImu))
        open('/home/tom/test.launch', 'w').write(GUI.MainWindow.prettify(root))
