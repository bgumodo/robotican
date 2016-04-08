__author__ = 'tom1231'
import json

SERVO = 1
BATTERY = 2
SWITCH = 3
IMU = 4
PPM = 5
GPS = 6
RELAY = 7
URF = 8
CLOSE_LOP_ONE = 9
CLOSE_LOP_TWO = 10
OPEN_LOP = 11
DIFF_CLOSE = 12
DIFF_OPEN = 13
EX_DEV = 14
HOKUYO = 15
OPRNNI = 16
USBCAM = 17
DIFF_CLOSE_FOUR = 18
ROBOT_MODEL = 19
SLAM = 20
Keyboard = 21
PPMReader = 22
JOYSTICK = 23
SMOOTHER = 24
LAUNCH = 25
NODE = 26
EMERGENCY_SWITCH = 27


class DeviceFrame():
    def __init__(self, devType, frame, data=None):
        self._devData = dict()
        self._otherDevs = data
        self._type = devType
        self._frame = frame
        self._isValid = False

        self._toSave = True

    def nameIsValid(self):
        for dev in self._otherDevs:
            if dev != self and dev.getName() == self.getName(): return False
        return True

    def getDevType(self):
        return self._type

    def getDevData(self):
        return self._devData

    def setType(self, devType):
        self._type = devType

    def isToSave(self):
        return self._toSave

    def setToSave(self, val):
        self._toSave = val

    def saveToFile(self, file):
        raise NotImplementedError

    def add(self):
        raise NotImplementedError

    def showDetails(self, items=None):
        raise NotImplementedError

    def getName(self):
        raise NotImplementedError

    def printDetails(self):
        raise NotImplementedError

    def isValid(self):
        return self._isValid

    def toDict(self):
        raise NotImplementedError

    def fromDict(self, data):
        raise NotImplementedError




