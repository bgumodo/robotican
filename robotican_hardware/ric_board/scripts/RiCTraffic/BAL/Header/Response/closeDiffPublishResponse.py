__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

ODOM_X = 10
ODOM_Y = 14
ODOM_THATE = 18
# TRAN_X = 24
# TRAN_Y = 28
# TRAN_Z = 32
TRAN_ROT_Z = 22
TRAN_ROT_W = 26
LINEAR_VEL = 30

class CloseDiffPublishRepose(RiCHeader):

    def __init__(self):
        RiCHeader.__init__(self)
        self._odomX = 0.0
        self._odomY = 0.0
        self._odomTheta = 0.0
        self._translationX = 0.0
        self._translationY = 0.0
        self._translationZ = 0.0
        self._translationRotationZ = 0.0
        self._translationRotationW = 0.0
        self._linearVelocity = 0.0

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < ODOM_X:
            bytes.append(data[self.index])
            self.index += 1
        self._odomX = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < ODOM_Y:
            bytes.append(data[self.index])
            self.index += 1
        self._odomY = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < ODOM_THATE:
            bytes.append(data[self.index])
            self.index += 1
        self._odomTheta = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < TRAN_X:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._translationX = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < TRAN_Y:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._translationY = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < TRAN_Z:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._translationZ = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < TRAN_ROT_Z:
            bytes.append(data[self.index])
            self.index += 1
        self._translationRotationZ = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < TRAN_ROT_W:
            bytes.append(data[self.index])
            self.index += 1
        self._translationRotationW = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < LINEAR_VEL:
            bytes.append(data[self.index])
            self.index += 1
        self._linearVelocity = struct.unpack('<f', bytes)[0]

    def getPublishData(self):
        return self._odomX,\
               self._odomY,\
               self._odomTheta,\
               self._translationX,\
               self._translationY,\
               self._translationZ,\
               self._translationRotationZ,\
               self._translationRotationW,\
               self._linearVelocity