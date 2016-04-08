__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader
import struct

REQ_LEN = 11

class CloseMotorRequest(RiCHeader):
    def __init__(self, motorNum, postion):
        RiCHeader.__init__(self)
        self._id = 6
        self._length = REQ_LEN
        self._des = 0x1001
        self._checkSum = 0
        self._motorId = motorNum
        self._position = postion
        self._checkSum = self.calCheckSum(self.dataTosend())


    def dataTosend(self):
        return RiCHeader.dataTosend(self) \
               + struct.pack('<B', self._motorId) \
               + struct.pack('<f', self._position)