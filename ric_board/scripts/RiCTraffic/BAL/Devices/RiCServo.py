import re
from threading import Thread
import rospy
import rostopic
from BAL.Handlers.keepAliveHandler import KeepAliveHandler
from BAL.Header.Requests.PublishRequest import PublishRequest
from BAL.Header.Requests.servoRequest import ServoRequest
from BAL.Header.Response.ParamBuildResponse import SERVO

__author__ = 'tom1231'
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32
from BAL.Interfaces.Device import Device


class RiCServo(Device):

    def __init__(self, params, servoNum, output):
        Device.__init__(self, params.getServoName(servoNum), output)
        self._servoNum = servoNum
        self._pub = Publisher('%s/Position' % self._name, Float32, queue_size=params.getServoPublishHz(servoNum))
        Subscriber('%s/command' % self._name, Float32, self.servoCallBack)
        #KeepAliveHandler('%s/Position' % self._name, Float32)
        self._haveRightToPublish = False

    def publish(self, data):
        msg = Float32()
        msg.data = data
        self._pub.publish(msg)

    def servoCallBack(self, recv):
        Thread(target=self.sendMsg, args=(recv,)).start()
        # TOOD: ServoRequest

    def getType(self): return SERVO

    def sendMsg(self, recv):
        position = recv.data
        msg = ServoRequest(self._servoNum, position)
        self._output.write(msg.dataTosend())

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]

            if not self._haveRightToPublish and subCheck == '':
                self._output.write(PublishRequest(SERVO, self._servoNum, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and subCheck == 'None':
                self._output.write(PublishRequest(SERVO, self._servoNum, False).dataTosend())
                self._haveRightToPublish = False
        except: pass
