__author__ = 'tom1231'
from BAL.Header.Response.ParamBuildResponse import ParamBuildResponse, PPM

MSG_LEN = 14


class PPMParamResponse(ParamBuildResponse):
    def __init__(self, param):
        ParamBuildResponse.__init__(self, PPM, 0, param.getPPMPubHz())
        self._length = MSG_LEN
        self._checkSum = 0
        self._checkSum = self.calCheckSum(self.dataTosend())