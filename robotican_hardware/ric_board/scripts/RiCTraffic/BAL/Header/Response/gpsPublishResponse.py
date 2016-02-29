__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader

LAT_PLACE = 10
LNG_PLACE = 14
METERS_PLACE = 18
HDOP_PLACE = 20
SAT_PLACE = 22
FIX_PLACE = 23


class GPSPublishResponse(RiCHeader):

    def __init__(self):
        RiCHeader.__init__(self)
        self._lat = 0.0
        self._lng = 0.0
        self._meters = 0.0
        self._HDOP = 0
        self._satellites = 0
        self._fix = 0

    def getLat(self): return self._lat

    def getLng(self): return self._lng

    def getMeters(self): return self._meters

    def getHDOP(self): return self._HDOP

    def getSatellites(self): return self._satellites

    def getFix(self): return self._fix

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < LAT_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._lat = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < LNG_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._lng = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < METERS_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._meters = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < HDOP_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._HDOP = struct.unpack('<h', bytes)[0]
        bytes = bytearray()
        while self.index < SAT_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._satellites = struct.unpack('<h', bytes)[0]
        bytes = bytearray()
        while self.index < FIX_PLACE:
            bytes.append(data[self.index])
            self.index += 1
        self._fix = struct.unpack('<b', bytes)[0]



