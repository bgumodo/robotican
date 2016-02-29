__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader
import struct

DEVICE_ID_PLACE = 7
REQ_LENGTH_PLACE = 8

class ACKResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._IdToAck = 0
        self._requestLength = 0

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < DEVICE_ID_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._IdToAck = struct.unpack('<b', bytes)[0]
        bytes = bytearray()
        while self.index < REQ_LENGTH_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._requestLength = struct.unpack('<b', bytes)[0]

    def getReqLen(self): return self._requestLength

    def getIdToAck(self): return self._IdToAck

