__author__ = 'tom1231'

NEED_TO_UPDATE = -1
SUG_TO_UPDATE = -2

class VersionError(Exception):
    def __init__(self, versionErrorCode):
        self._errorCode = versionErrorCode

    def __str__(self):
        return str(self._errorCode)

    def getCode(self):
        return self._errorCode

