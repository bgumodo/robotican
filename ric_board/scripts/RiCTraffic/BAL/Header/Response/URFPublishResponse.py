__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

ID_LEN = 7
RANGE_LEN = 11

class URFPublishResponse(RiCHeader):

    def __init__(self):
        RiCHeader.__init__(self)
        self._urfId = 0
        self._range = 0.0

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < ID_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._urfId = struct.unpack('<B', bytes)[0]
        bytes = bytearray()
        while self.index < RANGE_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._range = struct.unpack('<f', bytes)[0]

    def getURFId(self): return self._urfId

    def getRange(self): return self._range


