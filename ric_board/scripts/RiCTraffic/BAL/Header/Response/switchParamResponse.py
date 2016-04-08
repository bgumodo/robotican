__author__ = 'tom1231'
import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, Button

RES_LEN = 18

class SwitchParamResponse(ParamBuildResponse):
    def __init__(self, devId, param):
        ParamBuildResponse.__init__(self, Button, devId, param.getSwitchPubHz(devId))
        self._length = RES_LEN
        self._checkSum = 0
        self._port = param.getSwitchPort(devId)

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) + struct.pack('<i', self._port)


