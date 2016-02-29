import struct

__author__ = 'tom1231'
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, EngineCL

RES_LEN = 82
EN_PORT1 = 1
EN_PORT2 = 2
EN_PORT3 = 3
EN_PORT4 = 4

class CloseLoopMotorBuildResponse(ParamBuildResponse):
    def dataTosend(self):
        return ParamBuildResponse.dataTosend(self) \
               + struct.pack('<I', self._LPFHz) \
               + struct.pack('<f', self._LPFAlpha) \
               + struct.pack('<I', self._driverAddress) \
               + struct.pack('<I', self._channel) \
               + struct.pack('<I', self._PIDHz) \
               + struct.pack('<f', self._KP) \
               + struct.pack('<f', self._KI) \
               + struct.pack('<f', self._KD) \
               + struct.pack('<f', self._maxSpeed) \
               + struct.pack('<I', self._CPR) \
               + struct.pack('<I', self._timeout) \
               + struct.pack('<I', self._encoderPinA) \
               + struct.pack('<I', self._encoderPinB) \
               + struct.pack('<I', self._motorType) \
               + struct.pack('<i', self._motorDirection) \
               + struct.pack('<i', self._motorDirectionE) \
               + struct.pack('<f', self._limit)

    def __init__(self, devId, param, type):
        ParamBuildResponse.__init__(self, type, devId, param.getCloseLoopMotorPubHz(devId))
        self._length = RES_LEN
        self._LPFHz = param.getCloseLoopMotorLPFHz(devId)
        self._LPFAlpha = param.getCloseLoopMotorLPFAlpha(devId)
        self._driverAddress = param.getCloseLoopMotorDriverAddress(devId)
        self._channel = param.getCloseLoopMotorChannel(devId)
        self._PIDHz = param.getCloseLoopMotorPIDHz(devId)
        self._KP = param.getCloseLoopMotorKp(devId)
        self._KI = param.getCloseLoopMotorKi(devId)
        self._KD = param.getCloseLoopMotorKd(devId)
        self._maxSpeed = param.getCloseLoopMotorMaxSpeed(devId)
        self._CPR = param.getCloseLoopMotorCpr(devId)
        self._timeout = param.getCloseLoopMotorTimeout(devId)
        self._timeout = param.getCloseLoopMotorTimeout(devId)
        self._limit = param.getCloseLoopMotorIntegralLimit(devId)
        port = param.getCloseLoopMotorEncoderPort(devId)
        if port == EN_PORT1:
            self._encoderPinA = 20
            self._encoderPinB = 21
        elif port == EN_PORT2:
            self._encoderPinA = 22
            self._encoderPinB = 23
        elif port == EN_PORT3:
            self._encoderPinA = 27
            self._encoderPinB = 28
        elif port == EN_PORT4:
            self._encoderPinA = 29
            self._encoderPinB = 30
        self._motorType = param.getCloseLoopMotorType(devId)
        self._motorDirection = param.getCloseLoopMotorDirection(devId)
        self._motorDirectionE = param.getCloseLoopMotorDirectionEncoder(devId)
        if param.getCloseLoopMotorEncoderType(devId) == 1:
            self._checkSum = 0
            self._checkSum = self.calCheckSum(self.dataTosend())

