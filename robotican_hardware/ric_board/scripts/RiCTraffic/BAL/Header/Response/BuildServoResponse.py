import struct

__author__ = 'tom1231'
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, SERVO


SERVO_LEN_MSG = 38

class BuildServoResponse(ParamBuildResponse):
    def __init__(self, devId, param):
        ParamBuildResponse.__init__(self, SERVO, devId, param.getServoPublishHz(devId))
        self._length = SERVO_LEN_MSG
        self._port = param.getServoPort(devId)
        self._min = param.getServoMin(devId)
        self._max = param.getServoMax(devId)
        self._a = param.getServoAParameter(devId)
        self._b = param.getServoBParameter(devId)
        self._initPos = param.getServoInitMove(devId)

        self._checkSum = 0
        self._checkSum = self.calCheckSum(self.dataTosend())

    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self)\
               + struct.pack('<i', self._port)\
               + struct.pack('<f', self._min)\
               + struct.pack('<f', self._max)\
               + struct.pack('<f', self._a)\
               + struct.pack('<f', self._b)\
               + struct.pack('<f', self._initPos)

    def getDevId(self):
        return self._devId






