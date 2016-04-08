__author__ = 'tom1231'
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, EngineOL
import struct

MSG_LEN = 30


class OpenLoopMotorParamResponse(ParamBuildResponse):

    def __init__(self, devId, param):
        ParamBuildResponse.__init__(self, EngineOL, devId, 0)
        self._length = MSG_LEN
        self._checkSum = 0
        self._address = param.getOpenLoopAddress(devId)
        self._channel = param.getOpenLoopChannel(devId)
        self._timeout = param.getOpenLoopTimeout(devId)
        self._max = param.getOpenLoopMax(devId)

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) \
               + struct.pack('<I', self._address) \
               + struct.pack('<I', self._channel) \
               + struct.pack('<I', self._timeout) \
               + struct.pack('<i', self._max)

