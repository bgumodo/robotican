import struct

__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader

SERVO_NUM_PLACE = 7
SERVO_POSITION_PLACE = 11


class ServoPublishResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._servoNum = 0
        self._position = 0.0

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < SERVO_NUM_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._servoNum = struct.unpack('<B', bytes)[0]
        bytes = bytearray()
        while self.index < SERVO_POSITION_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._position = struct.unpack('<f', bytes)[0]

    def getServoNum(self): return self._servoNum

    def getServoPos(self): return self._position

