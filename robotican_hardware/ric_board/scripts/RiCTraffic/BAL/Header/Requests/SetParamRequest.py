__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
from BAL.Handlers.incomingHandler import PSET_REQ

MSG_LEN = 13

class SetParamRequest(RiCHeader):
    def __init__(self, devId, devType, fieldNum, val):
        RiCHeader.__init__(self)

        self._id = PSET_REQ
        self._length = MSG_LEN
        self._des = 0x1001
        self._checkSum = 0

        self._devId = devId         # byte
        self._devType = devType     # byte
        self._fieldNum = fieldNum   # byte
        self._val = val             # float

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return RiCHeader.dataTosend(self) \
               + struct.pack('<B', self._devId) \
               + struct.pack('<B', self._devType)\
               + struct.pack('<B', self._fieldNum)\
               + struct.pack('<f', self._val)


