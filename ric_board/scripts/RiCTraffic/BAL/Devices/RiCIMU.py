import re
from threading import Thread
import rospy
import rostopic
from BAL.Handlers.keepAliveHandler import KeepAliveHandler
from BAL.Header.Requests.PublishRequest import PublishRequest
from BAL.Header.Requests.imuRequest import IMURequest
from BAL.Header.Response.ParamBuildResponse import IMU

__author__ = 'tom1231'
from rospy import Publisher, Service
from sensor_msgs.msg import Imu, MagneticField
from BAL.Interfaces.Device import Device
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from math import pi
from ric_board.srv._calibIMU import calibIMU, calibIMURequest
from ric_board.msg._imuCalib import imuCalib

CALIB_ID = 1
PUB_ID = 2

class RiCIMU(Device):
    def __init__(self, param, output):
        Device.__init__(self, param.getIMUName(), output)
        self._frameId = param.getIMUFrameId()
        self._angle = param.getIMUOrientation()
        self._pub = Publisher('%s_AGQ' % self._name, Imu, queue_size=param.getIMUPubHz())
        self._pubMag = Publisher('%s_M' % self._name, MagneticField, queue_size=param.getIMUPubHz())
        self._pubCalib = Publisher('/imu_calib_publisher', imuCalib, queue_size=param.getIMUPubHz())
        Service('/imu_Calibration', calibIMU, self.serviceCallBack)
        self._haveRightToPublish = False
        self._calib = False
        self._xMax = 0.0
        self._yMax = 0.0
        self._zMax = 0.0
        self._xMin = 0.0
        self._yMin = 0.0
        self._zMin = 0.0
        #KeepAliveHandler('%s_AGQ' % self._name, Imu)

    def getType(self):
        return IMU

    def serviceCallBack(self, request):
        validStatus = True

        if request.status == calibIMURequest.START_CALIB:
            rospy.loginfo("Calibration begin")
            self._output.write(IMURequest(calibIMURequest.START_CALIB).dataTosend())
            self._calib = True

            self._xMax = request.xMax
            self._yMax = request.yMax
            self._zMax = request.zMax
            self._xMin = request.xMin
            self._yMin = request.yMin
            self._zMin = request.zMin

        elif request.status == calibIMURequest.STOP_CALIB:
            rospy.loginfo("Calibration end")
            self._output.write(IMURequest(calibIMURequest.STOP_CALIB).dataTosend())
            rospy.loginfo("Calibration setting saved")
            self._calib = False

        else:
            validStatus = False
        return {'ack': validStatus}

    def publish(self, data):
        if not self._calib and data.getImuMsgId() == PUB_ID:
            q = data.getOrientation()
            roll, pitch, yaw = euler_from_quaternion((q.y, q.z, q.w, q.x))
            #array = quaternion_from_euler(yaw + (self._angle * pi / 180), roll, pitch)
            #array = quaternion_from_euler(yaw + (self._angle * pi / 180), -1 (roll - 180),  -1 * pitch)
            array = quaternion_from_euler((yaw + (self._angle * pi / 180)), roll, -1 * pitch)
            res = Quaternion()
            res.w = array[0]
            res.x = array[1]
            res.y = array[2]
            res.z = array[3]
            
            msg = Imu()
            msg.header.frame_id = self._frameId
            msg.header.stamp = rospy.get_rostime()
            msg.orientation = res
            msg.linear_acceleration = data.getAcceleration()
            msg.angular_velocity = data.getVelocity()

            magMsg = MagneticField()
            magMsg.header.frame_id = self._frameId
            magMsg.header.stamp = rospy.get_rostime()
            magMsg.magnetic_field = data.getMagnetometer()

            self._pub.publish(msg)
            self._pubMag.publish(magMsg)

        elif data.getImuMsgId() == CALIB_ID:
            x, y, z = data.getValues()
            msg = imuCalib()

            if x > self._xMax:
                self._xMax = x
            if x < self._xMin:
                self._xMin = x

            if y > self._yMax:
                self._yMax = y
            if y < self._yMin:
                self._yMin = y

            if z > self._zMax:
                self._zMax = z
            if z < self._zMin:
                self._zMin = z


            msg.x.data = x
            msg.x.max = self._xMax
            msg.x.min = self._xMin

            msg.y.data = y
            msg.y.max = self._yMax
            msg.y.min = self._yMin

            msg.z.data = z
            msg.z.max = self._zMax
            msg.z.min = self._zMin

            self._pubCalib.publish(msg)

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]
            subMagCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pubMag.name)).group(0).split(': ')[1]
            subCalibCheck = \
            re.search('Subscribers:.*', rostopic.get_info_text(self._pubCalib.name)).group(0).split(': ')[1]

            if not self._haveRightToPublish and (subCheck == '' or subMagCheck == '' or subCalibCheck == ''):
                self._output.write(PublishRequest(IMU, 0, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and (
                        subCheck == 'None' and subMagCheck == 'None' and subCalibCheck == 'None'):
                self._output.write(PublishRequest(IMU, 0, False).dataTosend())
                self._haveRightToPublish = False
        except:
            pass
