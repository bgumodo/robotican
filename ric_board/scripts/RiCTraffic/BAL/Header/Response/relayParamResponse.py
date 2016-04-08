__author__ = 'tom1231'
import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, Rely

MSG_LEN = 18

class RelayParamResponse(ParamBuildResponse):

    def __init__(self, devId , param):
        ParamBuildResponse.__init__(self, Rely, devId, 0)
        self._length = MSG_LEN
        self._checkSum = 0

        self._port = param.getRelayPort(devId)

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) + struct.pack('<i', self._port)


