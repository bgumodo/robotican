import struct

HEADER_START = 0xfe
CONFIG_HEADER = 0xfc
LOAD_HEADER = 0xfb
DONE_HEADER = 0xfa

TAKEOVER_HEADER_SIZE = 3


class TakeoverHeader(object):
    def __init__(self, takeover_id, checksum=0):
        super(TakeoverHeader, self).__init__()
        self._takeover_id = takeover_id
        self._checksum = checksum

    def to_bytes(self):
        return struct.pack('<B', HEADER_START) \
               + struct.pack('<B', self._takeover_id) \
               + struct.pack('<H', self._checksum)

    def cal_checksum(self):
        data = self.to_bytes()[1:]
        checksum = 0
        for byt in data:
            checksum += ord(byt)

        return checksum

    def get_checksum(self):
        return self._checksum

    def set_checksum(self, value):
        self._checksum = value

    def get_id(self):
        return self._takeover_id

    def get_length(self):
        return TAKEOVER_HEADER_SIZE

    def pkg_ok(self):
        prev_checksum = self.get_checksum()
        self.set_checksum(0)
        self.set_checksum(self.cal_checksum())
        return prev_checksum == self.get_checksum()

    def convert_to_pkg(self, raw_data):
        self._takeover_id = struct.unpack('<B', bytearray(raw_data[0]))[0]
        self._checksum = struct.unpack('<H', bytearray(raw_data[1:3]))[0]
