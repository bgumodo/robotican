import re
from threading import Thread
import rostopic
from BAL.Handlers.keepAliveHandler import KeepAliveHandler
from BAL.Header.Requests.PublishRequest import PublishRequest

__author__ = 'tom1231'
import rospy
from rospy import Publisher
from ric_board.msg import PPM
from BAL.Interfaces.Device import Device


class RiCPPM(Device):
    def __init__(self, param, output):
        Device.__init__(self, param.getPPMName(), output)
        self._pub = Publisher('%s' % self._name, PPM, queue_size=param.getPPMPubHz())
        self._haveRightToPublish = False
        #KeepAliveHandler(self._name, PPM)

    def getType(self): return PPM

    def publish(self, data):
        msg = PPM()
        msg.channels = data.getChannels()
        self._pub.publish(msg)

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]

            if not self._haveRightToPublish and subCheck == '':
                self._output.write(PublishRequest(7, 0, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and subCheck == 'None':
                self._output.write(PublishRequest(7, 0, False).dataTosend())
                self._haveRightToPublish = False
        except: pass
