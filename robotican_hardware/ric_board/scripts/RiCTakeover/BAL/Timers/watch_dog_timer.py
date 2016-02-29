import time
__author__ = 'caja'


class WatchDogTimer(object):
    @staticmethod
    def now():
        return int(round(time.time() * 1000))

    def __init__(self, timeout):
        self._wd_time = WatchDogTimer.now()
        self._timeout = timeout
        self._is_timeout = False

    def check_if_timeout(self, now):
        if not self._is_timeout:
            self._is_timeout = (now - self._wd_time) > self._timeout
        return self._is_timeout

    def set_watch_dog(self, now):
        self._wd_time = now

    def is_timeout(self):
        return self._is_timeout