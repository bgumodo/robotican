import struct
import threading
from BAL.Timers.watch_dog_timer import WatchDogTimer
from BAL.protocol.packages.channel_respond import ChannelRespond
from BAL.protocol.packages.clib_status import ClibStatus, CANCEL, SAVE, RESET
from BAL.protocol.serial_hadler import SerialHandler
from BAL.protocol.packages.header import CONFIG_HEADER, LOAD_HEADER, HEADER_START, DONE_HEADER
from BAL.protocol.packages.channel_to_takeover import ChannelToTakeover, CHANNEL_TO_TAKEOVER
from BAL.protocol.packages.motor_to_takeover import MotorToTakeover, MOTOR_TO_TAKEOVER


class ProtocolHandler(object):
    def __init__(self, dev):
        self._serial_handler = SerialHandler(dev)
        self._forward_clib = False
        self._turn_clib = False

        self._lock = threading.RLock()

    def is_forward_clib(self):
        with self._lock:
            return self._forward_clib

    def set_forward_clib(self, value):
        with self._lock:
            self._forward_clib = value

    def is_turn_clib(self):
        with self._lock:
            return self._turn_clib

    def set_turn_clib(self, value):
        with self._lock:
            self._turn_clib = value

    def in_config_mode(self):
        self._serial_handler.write(struct.pack('<B', CONFIG_HEADER))

    def send_clib_status(self, status):
        self._serial_handler.write(ClibStatus(status).to_bytes())

    def load_channel_to_takeover(self):
        res = ChannelToTakeover()

        to_quit = False
        while not to_quit:
            self.wait_for_header()
            res.convert_to_pkg(self._serial_handler.read(res.get_length()))
            if res.get_id() == CHANNEL_TO_TAKEOVER:
                to_quit = True

        return res

    def load_motor_to_takeover(self):
        res = MotorToTakeover()

        to_quit = False
        while not to_quit:
            self.wait_for_header()
            res.convert_to_pkg(self._serial_handler.read(res.get_length()))
            if res.get_id() == MOTOR_TO_TAKEOVER:
                to_quit = True
        return res

    def send_channel_to_takeover_pkg(self, ch, listen_mode, value, status):
        self._serial_handler.write(ChannelToTakeover(ch, listen_mode, value, status).to_bytes())

    def wait_for_header(self):
        while ord(self._serial_handler.read(1)) != HEADER_START: pass

    def check_header_or_done(self):
        while True:
            read = ord(self._serial_handler.read(1))
            if read == HEADER_START:
                return True
            elif read == DONE_HEADER:
                return False

    def send_motor_to_takeover_and_clib(self, ch, max_command, pin, tab, min_joy, max_joy, to_clib, deadband, reverse, sp):
        self.set_forward_clib(True)
        self._serial_handler.write(MotorToTakeover(ch, max_command, pin, tab, min_joy, 0, max_joy, to_clib, deadband, reverse).to_bytes())
        self.wait_for_header()

        while self.check_header_or_done():
            res = ChannelRespond()
            res.convert_to_pkg(self._serial_handler.read(res.get_length()))
            sp.setValue(res.get_value())
            if min_joy > res.get_value():
                min_joy = res.get_value()
                sp.setMinimum(min_joy)
            elif max_joy < res.get_value():
                max_joy = res.get_value()
                sp.setMaximum(max_joy)

        self._serial_handler.flush()

    def send_motor_to_takeover(self, ch, max_command, pin, tab, min_joy, mid_joy, max_joy, to_clib, deadband, reverse):

        self._serial_handler.write(MotorToTakeover(ch, max_command, pin, tab, min_joy, mid_joy, max_joy, to_clib, deadband, reverse).to_bytes())

    def send_read_param(self):
        self._serial_handler.write(struct.pack('<B', LOAD_HEADER))

    def got_done(self):
        wd = WatchDogTimer(3000)
        now = WatchDogTimer.now()
        while not self._serial_handler.available() and not wd.check_if_timeout(now):
            now = WatchDogTimer.now()

        if not wd.is_timeout():
            return ord(self._serial_handler.read(1)) == DONE_HEADER
        return False