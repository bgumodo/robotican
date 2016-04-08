__author__ = 'tom1231'
import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse

RES_LEN = 18

class URFParamResponse(ParamBuildResponse):
    def __init__(self, type, devId, param):
        ParamBuildResponse.__init__(self, type, devId, param.getURFPubHz(devId))
        self._length = RES_LEN
        self._checkSum = 0
        self._portNum = param.getURFPort(devId)

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) + struct.pack('<i', self._portNum)

