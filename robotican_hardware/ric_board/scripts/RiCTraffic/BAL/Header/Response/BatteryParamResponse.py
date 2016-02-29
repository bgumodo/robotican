from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, Battery
import struct

__author__ = 'tom1231'

MSG_LEN = 18

class BatteryParamResponse(ParamBuildResponse):

    def __init__(self, param):
        ParamBuildResponse.__init__(self, Battery, 0, param.getBatteryPubHz())
        self._length = MSG_LEN
        self._checkSum = 0
        self._voltageDividerRatio = param.getBatteryVoltageDividerRatio()

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) \
               + struct.pack('<f', self._voltageDividerRatio)

