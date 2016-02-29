__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

REQ_ID = 81
REQ_LEN = 18


class CloseDiffSetOdomRequest(RiCHeader):
    def __init__(self, x, y, theta):
        RiCHeader.__init__(self)
        self._id = REQ_ID
        self._length = REQ_LEN
        self._des = 0x1001
        self._checkSum = 0
        self._x = x
        self._y = y
        self._theta = theta
        self._checkSum = self.calCheckSum(self.dataTosend())


    def dataTosend(self):
        return RiCHeader.dataTosend(self) \
               + struct.pack('<f', self._x) \
               + struct.pack('<f', self._y) \
               + struct.pack('<f', self._theta)


