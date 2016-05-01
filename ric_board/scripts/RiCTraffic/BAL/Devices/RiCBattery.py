
from threading import Thread
import time
import rospy
import rostopic
from BAL.Header.Requests.PublishRequest import PublishRequest
from BAL.Header.Response.ParamBuildResponse import Battery

__author__ = 'tom1231'
import re
from rospy import Publisher
from ric_board.msg import Battery
from BAL.Interfaces.Device import Device
from BAL.Handlers.keepAliveHandler import KeepAliveHandler
from BAL.RiCParam.RiCParam import RiCParam
class RiCBattery(Device):
    def __init__(self, param, output):
        """

        :param param:
        :type param: RiCParam
        :param output:
        :return:
        """
        Device.__init__(self, param.getBatteryName(), output)
        self._pub = Publisher('%s' % self._name, Battery, queue_size=param.getBatteryPubHz())
        self._haveRightToPublish = False

        self._min = param.getBatteryMin()
        self._max = param.getBatteryMax()

        #KeepAliveHandler(self._name, Float32)

    def publish(self, data):
        msg = Battery()
        msg.data = data
        msg.min = self._min
        msg.max = self._max

        self._pub.publish(msg)

    def getType(self): return Battery

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]

            if not self._haveRightToPublish and subCheck == '':
                self._output.write(PublishRequest(Battery, 0, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and subCheck == 'None':
                self._output.write(PublishRequest(Battery, 0, False).dataTosend())
                self._haveRightToPublish = False
        except: pass
