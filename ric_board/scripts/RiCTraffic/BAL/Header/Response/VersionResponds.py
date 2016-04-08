__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

VER_PLACE = 10

class VersionResponds(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._ver = 0.0

    def getVersion(self):
        return self._ver

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < VER_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._ver = struct.unpack('<f', bytes)[0]


