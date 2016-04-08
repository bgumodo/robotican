__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

BAT_STAT = 10


class BatteryPublishResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._status = 0.0

    def getStatus(self):
        return self._status

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < BAT_STAT:
            bytes.append(data[self.index])
            self.index += 1
        self._status = struct.unpack('<f', bytes)[0]

