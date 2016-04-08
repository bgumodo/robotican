__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
from BAL.Handlers.incomingHandler import RELAY_REQ

MSG_LEN = 8

class RelayRequest(RiCHeader):

    def __init__(self, relayNum, status):
        RiCHeader.__init__(self)
        self._id = RELAY_REQ
        self._length = MSG_LEN
        self._des = 0x1001
        self._checkSum = 0
        self._relayNum = relayNum
        self._status = status

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return RiCHeader.dataTosend(self) + struct.pack('<B', self._relayNum) \
               + struct.pack('<?', self._status)

