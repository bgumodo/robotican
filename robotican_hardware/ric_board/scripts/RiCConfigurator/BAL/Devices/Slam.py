__author__ = 'tom1231'
from PyQt4.QtGui import *
from BAL.Interface.DeviceFrame import DeviceFrame, EX_DEV, SLAM
from lxml.etree import Element, SubElement, XML


class Slam(DeviceFrame):
    def __init__(self, frame, data):
        DeviceFrame.__init__(self, EX_DEV, frame, data)
        self._tf_map_scanmatch_transform_frame_name = 'scanmatcher_frame'
        self._base_frame = 'base_link'
        self._odom_frame = 'odom_link'
        self._map_frame = 'map'
        self._scan_topic = 'scan'

    def toDict(self):
        data = dict()

        data['type'] = SLAM
        data['tf'] = self._tf_map_scanmatch_transform_frame_name
        data['base'] = self._base_frame
        data['odom'] = self._odom_frame
        data['map'] = self._map_frame
        data['scan'] = self._scan_topic

        return data

    def showDetails(self, items=None):
        self.tf_map_scanmatch_transform_frame_name = QLineEdit(self._tf_map_scanmatch_transform_frame_name)
        self.base_frame = QLineEdit(self._base_frame)
        self.odom_frame = QLineEdit(self._odom_frame)
        self.map_frame = QLineEdit(self._map_frame)
        self.scan_topic = QLineEdit(self._scan_topic)

        self._frame.layout().addRow(QLabel('Tf map scan match: '), self.tf_map_scanmatch_transform_frame_name)
        self._frame.layout().addRow(QLabel('Base frame: '), self.base_frame)
        self._frame.layout().addRow(QLabel('Odometry frame: '), self.odom_frame)
        self._frame.layout().addRow(QLabel('Map frame: '), self.map_frame)
        self._frame.layout().addRow(QLabel('Scan topic: '), self.scan_topic)


    def printDetails(self):
        self._frame.layout().addRow(QLabel('Tf map scan match: '), QLabel(self._tf_map_scanmatch_transform_frame_name))
        self._frame.layout().addRow(QLabel('Base frame: '), QLabel(self._base_frame))
        self._frame.layout().addRow(QLabel('Odometry frame: '), QLabel(self._odom_frame))
        self._frame.layout().addRow(QLabel('Map frame: '), QLabel(self._map_frame))
        self._frame.layout().addRow(QLabel('Scan topic: '), QLabel(self._scan_topic))

    def getName(self):
        return "SLAM"

    def saveToFile(self, parent):
        element = SubElement(parent, 'include', {
            'file': '$(find ric_board)/launch/hector_slam.launch'
        })

        SubElement(element, 'arg', {
            'name': 'tf_map_scanmatch_transform_frame_name',
            'value': self._tf_map_scanmatch_transform_frame_name
        })
        SubElement(element, 'arg', {
            'name': 'base_frame',
            'value': self._base_frame
        })
        SubElement(element, 'arg', {
            'name': 'odom_frame',
            'value': self._odom_frame
        })
        SubElement(element, 'arg', {
            'name': 'map_frame',
            'value': self._map_frame
        })
        SubElement(element, 'arg', {
            'name': 'pub_map_odom_transform',
            'default': 'true'
        })
        SubElement(element, 'arg', {
            'name': 'scan_subscriber_queue_size',
            'default': '5'
        })
        SubElement(element, 'arg', {
            'name': 'scan_topic',
            'value': self._scan_topic
        })
        SubElement(element, 'arg', {
            'name': 'map_size',
            'default': '2048'
        })

    def add(self):
        if not self.nameIsValid():
            QMessageBox.critical(self._frame, "Error", "Name already taken.")
            self._isValid = False
            return
        self._isValid = True
        self._tf_map_scanmatch_transform_frame_name = str(self.tf_map_scanmatch_transform_frame_name.text())
        self._base_frame = str(self.base_frame.text())
        self._odom_frame = str(self.odom_frame.text())
        self._map_frame = str(self.map_frame.text())
        self._scan_topic = str(self.scan_topic.text())

    def fromDict(self, data):
        self._tf_map_scanmatch_transform_frame_name = data['tf']
        self._base_frame = data['base']
        self._odom_frame = data['odom']
        self._map_frame = data['map']
        self._scan_topic = data['scan']