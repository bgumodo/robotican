__author__ = 'tom1231'
import struct
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse,IMU

RES_LEN = 21 # -2

class IMUParamResponse(ParamBuildResponse):

    def __init__(self, param):
        ParamBuildResponse.__init__(self, IMU, 0, param.getIMUPubHz())
        self._length = RES_LEN
        self._checkSum = 0

        self._camp = param.getIMUCamp()
        self._fusionHz = param.getIMUFusionHz()
        self._enableFuseGyro = param.isIMUFuseGyro()

        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self)\
               + struct.pack('<f', self._camp) \
               + struct.pack('<H', self._fusionHz)\
               + struct.pack('<?', self._enableFuseGyro)


