import re
from threading import Thread
import rostopic
from BAL.Handlers.keepAliveHandler import KeepAliveHandler
from BAL.Header.Requests.PublishRequest import PublishRequest
from BAL.Header.Response.ParamBuildResponse import URF_HRLV

__author__ = 'tom1231'
from BAL.Interfaces.Device import Device
from rospy import Publisher
import rospy
from sensor_msgs.msg import Range

MIN_RANGE_URF_LV_MaxSonar = 0.16
MAX_RANGE_URF_LV_MaxSonar = 6.45
FIELD_OF_VIEW_URF_LV_MaxSonar = 0.7

MIN_RANGE_URF_HRLV_MaxSonar = 0.3
MAX_RANGE_URF_HRLV_MaxSonar = 5.0
FIELD_OF_VIEW_URF_HRLV_MaxSonar = 0.7

class RiCURF(Device):

    def __init__(self, devId, param, output):
        Device.__init__(self, param.getURFName(devId), output)
        self._urfType = param.getURFType(devId)
        self._frameId = param.getURFFrameId(devId)
        self._pub = Publisher('%s' % self._name, Range, queue_size=param.getURFPubHz(devId))
        #KeepAliveHandler(self._name, Range)
        self._devId = devId

        self._haveRightToPublish = False

    def getType(self): return self._urfType

    def publish(self, data):
        msg = Range()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = self._frameId
        if self._urfType == URF_HRLV:
            msg.min_range = MIN_RANGE_URF_HRLV_MaxSonar
            msg.max_range = MAX_RANGE_URF_HRLV_MaxSonar
            msg.field_of_view = FIELD_OF_VIEW_URF_HRLV_MaxSonar
        else:
            msg.min_range = MIN_RANGE_URF_LV_MaxSonar
            msg.max_range = MAX_RANGE_URF_LV_MaxSonar
            msg.field_of_view = FIELD_OF_VIEW_URF_LV_MaxSonar
        msg.radiation_type = Range.ULTRASOUND
        msg.range = data
        self._pub.publish(msg)

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]

            if not self._haveRightToPublish and subCheck == '':
                self._output.write(PublishRequest(self.getType(), self._devId, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and subCheck == 'None':
                self._output.write(PublishRequest(self.getType(), self._devId, False).dataTosend())
                self._haveRightToPublish = False
        except: pass