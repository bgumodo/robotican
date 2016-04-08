__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader
import struct

REQ_ID = 2
REQ_LEN = 11

class ServoRequest(RiCHeader):
    def dataTosend(self):
        return RiCHeader.dataTosend(self)\
               + struct.pack('<B', self._servoId)\
               + struct.pack('<f', self._position)

    def __init__(self, servoId, position):
        RiCHeader.__init__(self)
        self._id = REQ_ID
        self._des = 0x1001
        self._length = REQ_LEN
        self._servoId = servoId
        self._position = position
        self._checkSum = 0
        self._checkSum = self.calCheckSum(self.dataTosend())


