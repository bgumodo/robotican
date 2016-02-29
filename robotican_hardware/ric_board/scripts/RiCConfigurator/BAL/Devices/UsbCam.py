import shlex
import subprocess
import re

__author__ = 'tom1231'
from PyQt4.QtCore import QUrl
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, USBCAM
from lxml.etree import Element, SubElement, XML


class UsbCam(DeviceFrame):

    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._name = 'RiC_CAM'
        self._output = 'screen'
        self._respawn = 'true'
        self._videoDevice = '/dev/video0'
        self._frameId = 'Head_Camera'
        self._format = 'yuyv'
        self._width = '640'
        self._height = '480'

    def fromDict(self, data):
        self._name = data['name']
        self._output = data['output']
        self._respawn = data['respawn']
        self._videoDevice = data['videoDevice']
        self._frameId = data['frameId']
        self._format = data['format']
        self._width = data['width']
        self._height = data['height']
        index = self._videoDevice.find('/dev/')
        if index != -1:
            self._videoDevice = self._videoDevice[index + 5:]

    def toDict(self):
        data = dict()

        data['type'] = USBCAM
        data['name'] = self._name
        data['output'] = self._output
        data['respawn'] = self._respawn
        data['videoDevice'] = self._videoDevice
        data['frameId'] = self._frameId
        data['format'] = self._format
        data['width'] = self._width
        data['height'] = self._height

        return data

    def printDetails(self):
        self._frame.layout().addRow(QLabel('name: '), QLabel(self._name))
        self._frame.layout().addRow(QLabel('Output: '), QLabel(self._output))
        self._frame.layout().addRow(QLabel('Respawn: '), QLabel(self._respawn))
        self._frame.layout().addRow(QLabel('Video device: '), QLabel(self._videoDevice))
        self._frame.layout().addRow(QLabel('Frame id: '), QLabel(self._frameId))
        self._frame.layout().addRow(QLabel('Pixel format: '), QLabel(self._format))
        self._frame.layout().addRow(QLabel('Image width: '), QLabel(self._width))
        self._frame.layout().addRow(QLabel('Image height: '), QLabel(self._height))

    def getName(self):
        return self._name

    def findItem(self):
        for i in xrange(self.videoDevice.count()):
            if self._videoDevice == str(self.videoDevice.itemText(i)):
                return i
        return -1

    def showDetails(self, items=None):
        self.name = QLineEdit(self._name)
        self.output = QLineEdit(self._output)
        self.respawn = QLineEdit(self._respawn)
        self.videoDevice = QComboBox()
        self.frameId = QLineEdit(self._frameId)
        self.format = QLineEdit(self._format)
        self.width = QLineEdit(self._width)
        self.height = QLineEdit(self._height)

        allDev = subprocess.check_output(shlex.split("ls /dev"))
        videoDevs = re.findall('video.*', allDev)

        self.videoDevice.addItems(videoDevs)
        index = self.findItem()
        if index != -1: self.videoDevice.setCurrentIndex(index)

        link = QLabel("<a href = http://wiki.ros.org/usb_cam> Usb Camera Wiki </a>")
        link.linkActivated.connect(self.onLink)

        self._frame.layout().addRow(QLabel('name: '), self.name)
        self._frame.layout().addRow(QLabel('Output: '), self.output)
        self._frame.layout().addRow(QLabel('Respawn: '), self.respawn)
        self._frame.layout().addRow(QLabel('Video device: '), self.videoDevice)
        self._frame.layout().addRow(QLabel('Frame id: '), self.frameId)
        self._frame.layout().addRow(QLabel('Pixel format: '), self.format)
        self._frame.layout().addRow(QLabel('Image width: '), self.width)
        self._frame.layout().addRow(QLabel('Image height: '), self.height)
        self._frame.layout().addRow(QLabel('More information: '), link)

    def onLink(self, URL):
        QDesktopServices().openUrl(QUrl(URL))

    def add(self):
        old = self._name
        self._name = str(self.name.text())

        if not self.nameIsValid():
            error = QErrorMessage()
            error.setWindowTitle("Same name error")
            error.showMessage("Name already taken.")
            error.exec_()
            self._name = old
            self._isValid = False
            return

        self._isValid = True
        self._name = str(self.name.text())
        self._output = str(self.output.text())
        self._respawn = str(self.respawn.text())
        self._videoDevice = str(self.videoDevice.currentText())
        self._format = str(self.format.text())
        self._frameId = str(self.frameId.text())
        self._width = str(self.width.text())
        self._height = str(self.height.text())

    def saveToFile(self, parent):
            nodeAt = {
                'pkg': 'usb_cam',
                'name': self._name,
                'output': self._output,
                'respawn': self._respawn,
                'type': 'usb_cam_node'
            }
            element = SubElement(parent, 'node', nodeAt)
            SubElement(element, 'param', {
                'name': 'video_device',
                'value': '/dev/%s' % self._videoDevice
            })
            SubElement(element, 'param', {
                'name': 'camera_frame_id',
                'value': self._frameId
            })
            SubElement(element, 'param', {
                'name': 'pixel_format',
                'value': self._format
            })
            SubElement(element, 'param', {
                'name': 'image_width',
                'value': self._width
            })
            SubElement(element, 'param', {
                'name': 'image_height',
                'value': self._height
            })