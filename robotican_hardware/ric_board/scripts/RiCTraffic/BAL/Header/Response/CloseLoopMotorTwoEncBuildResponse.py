import struct
from BAL.Header.Response.closeLoopMotorBuildResponse import CloseLoopMotorBuildResponse, EN_PORT1, EN_PORT2, EN_PORT3
from BAL.Header.Response.ParamBuildResponse import EngineCL2

__author__ = 'tom1231'

LEN_MSG = 90


class CloseLoopMotorTwoEncBuildResponse(CloseLoopMotorBuildResponse):
    def __init__(self, devId, param, type):
        CloseLoopMotorBuildResponse.__init__(self, devId, param, type)
        self._length = LEN_MSG
        port = param.getCloseLoopMotorPort2(devId)

        if port == EN_PORT1:
            self._encoderPin2A = 20
            self._encoderPin2B = 21
        elif port == EN_PORT2:
            self._encoderPin2A = 22
            self._encoderPin2B = 23
        elif port == EN_PORT3:
            self._encoderPin2A = 27
            self._encoderPin2B = 28
        else:
            self._encoderPin2A = 29
            self._encoderPin2B = 30

        self._checkSum = 0
        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return CloseLoopMotorBuildResponse.dataTosend(self) \
               + struct.pack('<I', self._encoderPin2A) \
               + struct.pack('<I', self._encoderPin2B)

