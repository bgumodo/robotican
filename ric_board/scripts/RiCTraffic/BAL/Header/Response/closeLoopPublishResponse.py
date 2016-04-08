__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader
import struct

MOTOR_ID = 7
MOTOR_RAD = 11
MOTOR_RAD_S = 15


class CloseLoopPublishResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._devId = 0
        self._rad = 0
        self._rad_s = 0

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < MOTOR_ID:
            bytes.append(data[self.index])
            self.index += 1
        self._devId = struct.unpack('<B', bytes)[0]
        bytes = bytearray()
        while self.index < MOTOR_RAD:
            bytes.append(data[self.index])
            self.index += 1
        self._rad = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < MOTOR_RAD_S:
            bytes.append(data[self.index])
            self.index += 1
        self._rad_s = struct.unpack('<f', bytes)[0]

    def getMotorId(self):
        return self._devId

    def getMotorRad(self):
        return self._rad

    def getMotorRadS(self):
        return self._rad_s

    def getMotorMsg(self):
        return [self.getMotorRad(), self.getMotorRadS()]