import re
from threading import Thread
import rospy
import rostopic
from BAL.Handlers.keepAliveHandler import KeepAliveHandler
from BAL.Header.Requests.PublishRequest import PublishRequest
from BAL.Header.Response.ParamBuildResponse import Button

__author__ = 'tom1231'
from rospy import Publisher
from BAL.Interfaces.Device import Device
from std_msgs.msg import Bool


class RiCSwitch(Device):
    def __init__(self, devId,param, output):
        Device.__init__(self, param.getSwitchName(devId), output)
        self._pub = Publisher('%s' % self._name, Bool, queue_size=param.getSwitchPubHz(devId))
        self._switchId = devId
        self._haveRightToPublish = False
       # KeepAliveHandler(self._name, Bool)

    def publish(self, data):
        msg = Bool()
        msg.data = data
        self._pub.publish(msg)

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]

            if not self._haveRightToPublish and subCheck == '':
                self._output.write(PublishRequest(Button, self._switchId, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and subCheck == 'None':
                self._output.write(PublishRequest(Button, self._switchId, False).dataTosend())
                self._haveRightToPublish = False
        except: pass

    def getType(self): return Button