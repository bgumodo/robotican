__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
from BAL.Handlers.incomingHandler import OPEN_LOOP_REQ

MSG_LEN = 11

class OpenLoopMotorRequest(RiCHeader):
    def __init__(self, motorNum, speed):
        RiCHeader.__init__(self)
        self._id = OPEN_LOOP_REQ
        self._length = MSG_LEN
        self._des = 0x1001
        self._checkSum = 0
        self._motorNum = motorNum
        self._speed = speed

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return RiCHeader.dataTosend(self) \
               + struct.pack('<B', self._motorNum) \
               + struct.pack('<f', self._speed)

