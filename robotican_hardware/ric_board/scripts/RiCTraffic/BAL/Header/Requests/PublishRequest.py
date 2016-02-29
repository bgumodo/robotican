__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
from BAL.Handlers.incomingHandler import RIG_REQ

MSG_LEN = 9

class PublishRequest(RiCHeader):
    def __init__(self, devType, devId, haveRight):
        RiCHeader.__init__(self)

        self._id = RIG_REQ
        self._length = MSG_LEN
        self._des = 0x1001
        self._checkSum = 0

        self._devType = devType
        self._devId = devId
        self._haveRight = haveRight

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return RiCHeader.dataTosend(self) + struct.pack('<B', self._devType)\
               + struct.pack('<B', self._devId) + struct.pack('<?', self._haveRight)


