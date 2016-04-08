__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

CHANNEL_1 = 10
CHANNEL_2 = 14
CHANNEL_3 = 18
CHANNEL_4 = 22
CHANNEL_5 = 26
CHANNEL_6 = 30
CHANNEL_7 = 34
CHANNEL_8 = 38


class PPMPublishResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._ch1 = 0.0
        self._ch2 = 0.0
        self._ch3 = 0.0
        self._ch4 = 0.0
        self._ch5 = 0.0
        self._ch6 = 0.0
        self._ch7 = 0.0
        self._ch8 = 0.0

    def getChannels(self):
        return [self._ch1, self._ch2, self._ch3, self._ch4, self._ch5, self._ch6, self._ch7, self._ch8]

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < CHANNEL_1:
            bytes.append(data[self.index])
            self.index += 1
        self._ch1 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_2:
            bytes.append(data[self.index])
            self.index += 1
        self._ch2 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_3:
            bytes.append(data[self.index])
            self.index += 1
        self._ch3 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_4:
            bytes.append(data[self.index])
            self.index += 1
        self._ch4 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_5:
            bytes.append(data[self.index])
            self.index += 1
        self._ch5 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_6:
            bytes.append(data[self.index])
            self.index += 1
        self._ch6 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_7:
            bytes.append(data[self.index])
            self.index += 1
        self._ch7 = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < CHANNEL_8:
            bytes.append(data[self.index])
            self.index += 1
        self._ch8 = struct.unpack('<f', bytes)[0]