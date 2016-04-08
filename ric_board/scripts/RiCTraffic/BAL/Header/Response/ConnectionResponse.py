import struct

__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader

RES_ID = 101

class ConnectionResponse(RiCHeader):

    def dataTosend(self):
        return RiCHeader.dataTosend(self) + struct.pack('<?', self._toConnect)

    def __init__(self, toConnect):
        RiCHeader.__init__(self)
        self._id = RES_ID
        self._des = 0x1001
        self._length = 7
        self._checkSum = 0
        self._toConnect = toConnect
        self._checkSum = self.calCheckSum(self.dataTosend())




