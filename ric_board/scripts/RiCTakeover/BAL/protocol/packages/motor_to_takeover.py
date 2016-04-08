import struct
from BAL.protocol.packages.header import TakeoverHeader

MOTOR_TO_TAKEOVER = 3
MOTOR_TO_TAKEOVER_SIZE = 17

REVERSE = -1
NORMAL = 1


class MotorToTakeover(TakeoverHeader):
    def __init__(self, ch=None, max_command=None, pin=None, tab=None, min_joy=None, mid_joy=None ,max_joy=None, to_clib=None, deadband=None, rev=None):
        super(MotorToTakeover, self).__init__(MOTOR_TO_TAKEOVER)
        self._ch = ch
        self._max_command = max_command
        self._pin = pin
        self._tab = tab
        self._min_joy = min_joy
        self._max_joy = max_joy
        self._mid_joy = mid_joy
        self._to_clib = to_clib
        self._deadband = deadband

        self._rev = rev
        if ch is not None:
            self.set_checksum(self.cal_checksum())

    def get_channel(self):
        return self._ch

    def get_max_command(self):
        return self._max_command

    def get_pin(self):
        return self._pin

    def get_tab(self):
        return self._tab

    def get_min_joystick(self):
        return self._min_joy

    def get_max_joystick(self):
        return self._max_joy

    def get_mid_joystick(self):
        return self._mid_joy

    def get_deadband(self):
        return self._deadband

    def to_clib(self):
        return self._to_clib

    def get_length(self):
        return super(MotorToTakeover, self).get_length() + MOTOR_TO_TAKEOVER_SIZE

    def is_reverse(self):
        return self._rev == REVERSE

    def to_bytes(self):
        return super(MotorToTakeover, self).to_bytes() \
               + struct.pack('<B', self._ch) \
               + struct.pack('<H', self._max_command) \
               + struct.pack('<B', self._pin) \
               + struct.pack('<B', self._tab) \
               + struct.pack('<H', self._min_joy) \
               + struct.pack('<H', self._mid_joy) \
               + struct.pack('<H', self._max_joy) \
               + struct.pack('<?', self._to_clib) \
               + struct.pack('<f', self._deadband) \
               + struct.pack('<b', self._rev)

    def convert_to_pkg(self, raw_data):
        super(MotorToTakeover, self).convert_to_pkg(raw_data)
        self._ch = struct.unpack('<B', bytearray(raw_data[3]))[0]
        self._max_command = struct.unpack('<H', bytearray(raw_data[4:6]))[0]
        self._pin = struct.unpack('<B', bytearray(raw_data[6]))[0]
        self._tab = struct.unpack('<B', bytearray(raw_data[7]))[0]
        self._min_joy = struct.unpack('<H', bytearray(raw_data[8:10]))[0]
        self._mid_joy = struct.unpack('<H', bytearray(raw_data[10:12]))[0]
        self._max_joy = struct.unpack('<H', bytearray(raw_data[12:14]))[0]
        self._to_clib = struct.unpack('<?', bytearray(raw_data[14]))[0]
        self._deadband = struct.unpack('<f', bytearray(raw_data[15:19]))[0]
        self._rev = struct.unpack('<b', bytearray(raw_data[19]))[0]





