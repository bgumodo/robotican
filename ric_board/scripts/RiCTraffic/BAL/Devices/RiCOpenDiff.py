from threading import Thread
from BAL.Header.Response.ParamBuildResponse import DiffDriverOL

__author__ = 'tom1231'
from BAL.Interfaces.Device import Device
from rospy import Subscriber
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

    # _right->setStartPoint(((v / _rWheel) + (_weigh * w / (2 * _rWheel))));
    # _left->setStartPoint(((v / _rWheel) - (_weigh * w / (2 * _rWheel))));
    # openLoopCallback

class RiCOpenDiff(Device):

    def __init__(self, param, motorL, motorR):
        Device.__init__(self, param.getCloseDiffName(), None)
        self._maxAng = param.getCloseDiffMaxAng()
        self._rWheel = param.getCloseDiffRWheel()
        self._width = param.getCloseDiffWidth()
        self._maxLin = param.getCloseDiffMaxLin()
        self._motorL = motorL
        self._motorR = motorR
        Subscriber('%s/command' % self._name, Twist, self.diffCallback)

    def sendMsg(self, msg):
        if msg.angular.z > self._maxAng:
            msg.angular.z = self._maxAng
        elif msg.angular.z < -self._maxAng:
            msg.angular.z = -self._maxAng
        if msg.linear.x > self._maxLin:
            msg.linear.x = self._maxLin
        elif msg.linear.x < -self._maxLin:
            msg.linear.x = -self._maxLin

        msgR = Float32()
        msgL = Float32()
        w_max = self._maxLin / self._rWheel

        resR = ((msg.linear.x / self._rWheel) + (self._width * msg.angular.z / (2 * self._rWheel)))
        resL = ((msg.linear.x / self._rWheel) - (self._width * msg.angular.z / (2 * self._rWheel)))

        msgR.data = resR / w_max
        self._motorR.openLoopCallback(msgR)

        msgL.data = resL / w_max
        self._motorL.openLoopCallback(msgL)

    def diffCallback(self, msg):
        Thread(target=self.sendMsg, args=(msg,)).start()

    def getType(self): return DiffDriverOL

    def publish(self, data):
        pass