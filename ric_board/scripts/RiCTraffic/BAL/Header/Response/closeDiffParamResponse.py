__author__ = 'tom1231'
import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, DiffDriverCL

MSG_LEN = 34

class CloseDiffParamResponse(ParamBuildResponse):

    def __init__(self, devId, param):
        ParamBuildResponse.__init__(self, DiffDriverCL, devId, param.getCloseDiffPubHz())
        self._length = MSG_LEN
        self._checkSum = 0
        self._rWheel = param.getCloseDiffRWheel()
        self._width = param.getCloseDiffWidth()
        self._motorL = param.getCloseDiffMotorL()
        self._motorR = param.getCloseDiffMotorR()
        self._slip = param.getCloseDiffSlip()
        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) \
               + struct.pack('<f', self._rWheel) \
               + struct.pack('<f', self._width) \
               + struct.pack('<i', self._motorL) \
               + struct.pack('<i', self._motorR) \
               + struct.pack('<f', self._slip)