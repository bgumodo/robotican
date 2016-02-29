import rospy
from BAL.Handlers.incomingDataHandler import IncomingDataHandler
from BAL.Interfaces.Runnable import Runnable
from threading import Condition

__author__ = 'tom1231'


class IncomingMsgHandler(Runnable):
    def __init__(self, devs, output):
        self._queue = []
        self._lock = Condition()
        self._devs = devs
        self._output = output
        self._close = False

    def getMsg(self):
        with self._lock:
            while len(self._queue) == 0:
                self._lock.wait()
                if self._close : return None
            return self._queue.pop(0)

    def close(self):
        with self._lock:
            self._close = True
            self._lock.notifyAll()

    def addMsg(self, msg):
        with self._lock:
            self._queue.append(msg)
            self._lock.notifyAll()

    def run(self):
        while not self._close:
            msg = self.getMsg()
            if msg is not None:
                IncomingDataHandler(msg, self._output, self._devs).run()


