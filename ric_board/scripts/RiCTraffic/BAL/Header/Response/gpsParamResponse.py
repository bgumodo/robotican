__author__ = 'tom1231'
import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, GPS

MSG_LEN = 18

class GPSParamResponse(ParamBuildResponse):
    def __init__(self, param):
        ParamBuildResponse.__init__(self, GPS, 0, param.getGpsPubHz())
        self._length = MSG_LEN
        self._checkSum = 0
        self._baudrate = param.getGpsBaudrate()

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) + struct.pack('<i', self._baudrate)

