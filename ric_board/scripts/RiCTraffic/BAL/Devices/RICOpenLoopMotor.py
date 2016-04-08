from threading import Thread
from BAL.Header.Requests.openLoopMotorRequest import OpenLoopMotorRequest
from BAL.Header.Response.ParamBuildResponse import EngineOL

__author__ = 'tom1231'
from rospy import Subscriber
from std_msgs.msg import Float32
from BAL.Interfaces.Device import Device


class OpenLoopMotor(Device):
    def __init__(self, motorId,param, output):
        Device.__init__(self, param.getOpenLoopName(motorId), output)
        self._id = motorId
        self._direction = param.getOpenLoopDirection(motorId)
        Subscriber('%s/command' % self._name, Float32, self.openLoopCallback)

    def publish(self, data): pass

    def getType(self): return EngineOL

    def openLoopCallback(self, msg):
        Thread(target=self.sendMsg, args=(msg,)).start()

    def sendMsg(self, msg):
        self._output.write(OpenLoopMotorRequest(self._id, msg.data * self._direction).dataTosend())