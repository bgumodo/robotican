import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse,EmergencySwitch

MSG_LEN = 16


class EmergencySwitchParamResponse(ParamBuildResponse):
    def __init__(self, switchNum, param):
        ParamBuildResponse.__init__(self, EmergencySwitch, switchNum, 0)
        self._length = MSG_LEN
        self._checkSum = 0

        self._listenToPin = param.getEmergencyPin(switchNum)
        self._status = param.getEmergencyState(switchNum)

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) \
               + struct.pack('<B', self._listenToPin) \
               + struct.pack('<B', self._status)


