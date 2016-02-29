from BAL.Header.RiCHeader import RiCHeader
import struct

DiffDriverCL = 0
Button = 1
GPS = 2
IMU = 3
IR = 4
EngineOL = 5
EngineCL = 6
PPM = 7
Rely = 8
SERVO = 9
URF_HRLV = 10
URF_LV = 11
EngineCL2 = 12
Battery = 13
CloseDiffFour = 14
DiffDriverOL = 15
EmergencySwitch = 16


class ParamBuildResponse(RiCHeader):
    def __init__(self, type, devId, pubHz):
        RiCHeader.__init__(self)
        self._id = 103
        self._des = 0x1001
        self._type = type
        self._devId = devId
        self._pubHz = pubHz

    def dataTosend(self):
        return RiCHeader.dataTosend(self) \
               + struct.pack('<h', self._type)\
               + struct.pack('<h', self._devId) \
               + struct.pack('<I', self._pubHz)


