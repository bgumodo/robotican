__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

REQ_ID = 8
REQ_LEN = 14


class CloseDiffRequest(RiCHeader):
    def __init__(self, angular, linear):
        RiCHeader.__init__(self)
        self._id = REQ_ID
        self._length = REQ_LEN
        self._des = 0x1001
        self._checkSum = 0
        self._angular = angular
        self._linear = linear
        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return RiCHeader.dataTosend(self)\
               + struct.pack('<f', self._angular) \
               + struct.pack('<f', self._linear)


