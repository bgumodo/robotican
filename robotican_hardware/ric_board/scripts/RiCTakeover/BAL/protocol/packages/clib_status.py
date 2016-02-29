import struct
from BAL.protocol.packages.header import TakeoverHeader

CLIB_STATUS = 4

SAVE = 1
CANCEL = 2
RESET = 3


class ClibStatus(TakeoverHeader):
    def __init__(self, status):
        super(ClibStatus, self).__init__(CLIB_STATUS)
        self._status = status

        self.set_checksum(self.cal_checksum())

    def get_status(self):
        return self._status

    def to_bytes(self):
        return super(ClibStatus, self).to_bytes() \
               + struct.pack('<B', self._status)


