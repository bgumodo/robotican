from BAL.Header.RiCHeader import RiCHeader

__author__ = 'tom'
import struct

X = 10
Y = 14
Z = 18




class ImuCalibResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0


    def getImuMsgId(self):
        return 1


    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()

        while self.index < X:
            bytes.append(data[self.index])
            self.index += 1

        self._x = struct.unpack('<f', bytes)[0]

        bytes = bytearray()

        while self.index < Y:
            bytes.append(data[self.index])
            self.index += 1

        self._y = struct.unpack('<f', bytes)[0]

        bytes = bytearray()

        while self.index < Z:
            bytes.append(data[self.index])
            self.index += 1

        self._z = struct.unpack('<f', bytes)[0]

    def getValues(self):

        return (
            self._x,
            self._y,
            self._z,
        )
