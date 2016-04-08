import struct
from BAL.protocol.packages.header import TakeoverHeader

CHANNEL_RESPOND = 1
CHANNEL_RESPOND_SIZE = 3


class ChannelRespond(TakeoverHeader):
    def __init__(self):
        super(ChannelRespond, self).__init__(CHANNEL_RESPOND)
        self._chNum = 0
        self._value = 0

    def get_chNum(self):
        return self._chNum

    def get_value(self):
        return self._value

    def get_length(self):
        return TakeoverHeader.get_length(self) + CHANNEL_RESPOND_SIZE

    def convert_to_pkg(self, raw_data):
        TakeoverHeader.convert_to_pkg(self, raw_data)
        self._chNum = struct.unpack('<B', bytearray(raw_data[3]))[0]
        self._value = struct.unpack('<H', bytearray(raw_data[4:6]))[0]




