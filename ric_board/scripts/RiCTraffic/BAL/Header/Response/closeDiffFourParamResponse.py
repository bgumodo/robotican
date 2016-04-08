import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, CloseDiffFour

__author__ = 'tom1231'
MSG_LEN = 42


class CloseDIffFourParamResponse(ParamBuildResponse):
    def __init__(self, devId, param):
        ParamBuildResponse.__init__(self, CloseDiffFour, devId, param.getCloseDiffPubHz())
        self._length = MSG_LEN
        self._checkSum = 0
        self._rWheel = param.getCloseDiffRWheel()
        self._width = param.getCloseDiffWidth()
        self._motorFL = param.getCloseDiffMotorFL()
        self._motorFR = param.getCloseDiffMotorFR()
        self._motorBL = param.getCloseDiffMotorBL()
        self._motorBR = param.getCloseDiffMotorBR()
        self._slip = param.getCloseDiffSlip()
        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) \
               + struct.pack('<f', self._rWheel) \
               + struct.pack('<f', self._width) \
               + struct.pack('<i', self._motorFL) \
               + struct.pack('<i', self._motorFR) \
               + struct.pack('<i', self._motorBL) \
               + struct.pack('<i', self._motorBR) \
               + struct.pack('<f', self._slip)