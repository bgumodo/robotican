import struct
from BAL.protocol.packages.header import TakeoverHeader

CHANNEL_TO_TAKEOVER = 2
CHANNEL_TO_TAKEOVER_SIZE = 5


class ChannelToTakeover(TakeoverHeader):
    ABOVE = 1
    BELOW = 2

    def __init__(self, ch=None, listen_mode=None, value=None, status=None):
        super(ChannelToTakeover, self).__init__(CHANNEL_TO_TAKEOVER)
        self._ch = ch
        self._listen_mode = listen_mode
        self._value = value
        self._status = status

        if ch is not None:
            self.set_checksum(self.cal_checksum())

    def to_bytes(self):
        return super(ChannelToTakeover, self).to_bytes() \
               + struct.pack('<B', self._ch) \
               + struct.pack('<B', self._listen_mode) \
               + struct.pack('<H', self._value) \
               + struct.pack('<?', self._status)

    def get_length(self):
        return super(ChannelToTakeover, self).get_length() + CHANNEL_TO_TAKEOVER_SIZE

    def get_channel(self):
        return self._ch

    def get_listen_mode(self):
        return self._listen_mode

    def get_value(self):
        return self._value

    def get_status(self):
        return self._status

    def convert_to_pkg(self, raw_data):
        super(ChannelToTakeover, self).convert_to_pkg(raw_data)
        self._ch = struct.unpack('<B', bytearray(raw_data[3]))[0]
        self._listen_mode = struct.unpack('<B', bytearray(raw_data[4]))[0]
        self._value = struct.unpack('<H', bytearray(raw_data[5:7]))[0]
        self._status = struct.unpack('<?', bytearray(raw_data[7]))[0]




