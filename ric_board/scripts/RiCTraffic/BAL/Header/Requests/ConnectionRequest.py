import struct

__author__ = 'tom1231'
from BAL.Header.RiCHeader import RiCHeader

CON_REQ_PLACE = 7

class ConnectionRequest(RiCHeader):

    def __init__(self):
        RiCHeader.__init__(self)
        self._requestForConnection = 0

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < CON_REQ_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._requestForConnection = struct.unpack('<?', bytes)[0]


    def toConnect(self): return self._requestForConnection