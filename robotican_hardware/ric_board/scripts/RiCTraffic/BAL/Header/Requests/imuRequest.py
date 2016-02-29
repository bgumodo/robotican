__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
from BAL.Handlers.incomingHandler import IMU_REQ

MSG_LEN = 7


class IMURequest(RiCHeader):
    def __init__(self, status):
        RiCHeader.__init__(self)

        self._id = IMU_REQ
        self._length = MSG_LEN
        self._des = 0x1001
        self._checkSum = 0

        self.status = status

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return RiCHeader.dataTosend(self) + struct.pack('<B', self.status)