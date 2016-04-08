from BAL.Header.Response.ConnectionResponse import ConnectionResponse
from BAL.Interfaces.Runnable import Runnable
from BAL.Handlers.incomingHandler import SERVO_RES, CON_REQ, MOTOR_RES, CLOSE_DIFF_RES, URF_RES, SWITCH_RES, IMU_RES, GPS_RES, \
    PPM_RES, BAT_RES, IMU_CLIB_RES
import rospy

__author__ = 'tom1231'


class IncomingDataHandler(Runnable):
    def __init__(self, data, output, devices):
        self._output = output
        self._data = data
        self._dev = devices

    def run(self):
        if self._data.checkPackage():
            if self._data.getId() == CON_REQ:
                if self._data.toConnect():
                    self._output.write(ConnectionResponse(True).dataTosend())
            elif self._data.getId() == SERVO_RES:
                self._dev['servos'][self._data.getServoNum()].publish(self._data.getServoPos())

            elif self._data.getId() == MOTOR_RES:
                self._dev['motorsCl'][self._data.getMotorId()].publish(self._data.getMotorMsg())
            elif self._data.getId() == URF_RES:
                self._dev['urf'][self._data.getURFId()].publish(self._data.getRange())
            elif self._data.getId() == CLOSE_DIFF_RES:
                self._dev['diff'][0].publish(self._data.getPublishData())
            elif self._data.getId() == SWITCH_RES:
                self._dev['switch'][self._data.getSwitchNum()].publish(self._data.getStatus())
            elif self._data.getId() in [IMU_RES, IMU_CLIB_RES]:
                self._dev['imu'][0].publish(self._data)
            elif self._data.getId() == GPS_RES:
                self._dev['gps'][0].publish(self._data)
            elif self._data.getId() == PPM_RES:
                self._dev['ppm'][0].publish(self._data)
            elif self._data.getId() == BAT_RES:
                self._dev['battery'][0].publish(self._data.getStatus())

        else:
            rospy.logdebug('CheckSum is not valid')



