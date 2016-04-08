from BAL.Header.Response.ParamBuildResponse import EngineCL

__author__ = 'tom1231'
from rospy import Service
from threading import Thread
from ric_board.msg import DevStatus, MonitorDevs
from ric_board.srv._get_devs import get_devs
from ric_board.srv._setParam import setParam


BUF_SIZE = 1024
PORT = 1900

class Server:
    def __init__(self, devs, params):
        statusDevs = []
        self._devs = devs

        for dev in devs['motorsCl']:
                motor = DevStatus()
                motor.devName = dev.getName()
                motor.type = EngineCL
                motor.values = [dev.getKp(), dev.getKi(), dev.getKd()]
                statusDevs.append(motor)

        self._monitor = MonitorDevs()
        self._monitor.devs = statusDevs
        Service('/devsOnline', get_devs, self.getDevsOnline)
        Service('/devsSetParam', setParam, self.setParam)

    def getDevsOnline(self, req):
        devs = MonitorDevs()

        if req.req:
            devs = self._monitor
            index = 0
            for dev in devs.devs:
                motor = self._devs['motorsCl'][index]
                dev.values = [motor.getKp(), motor.getKi(), motor.getKd()]
                index += 1

        return {'devs': devs}

    def setParam(self, request):
        for motor in self._devs['motorsCl']:
            if request.dev.devName == motor.getName():
                motor.sendSetParam(request.dev.values)
                break
        return {'ack': True}

