import serial
import threading


class SerialHandler(object):
    def __init__(self, dev):
        self._ser = serial.Serial('/dev/' + dev)
        self._input_lock = threading.Condition()
        self._output_lock = threading.Condition()

    def read(self, size):
        with self._input_lock:
            return self._ser.read(size)

    def write(self, data):
        with self._output_lock:
            self._ser.write(data)

    def flush(self):
        with self._output_lock:
            while self._ser.inWaiting() > 0:
                print self._ser.inWaiting()
                self._ser.read(1)

    def available(self):
        with self._input_lock:
            return self._ser.inWaiting()
