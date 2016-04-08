__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
ID_REQ = 7
LEN_REQ = 7


class FinishBuildingRequest(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._id = ID_REQ
        self._length = LEN_REQ
        self._des = 0
        self._checkSum = 0
        self._finish = True
        self._checkSum = self.calCheckSum(self.dataTosend())


    def dataTosend(self):
        return RiCHeader.dataTosend(self) + struct.pack('<?', self._finish)