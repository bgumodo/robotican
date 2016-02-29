import serial
import time
import roslaunch
import rosnode
from BAL.Devices.DevicesBuilder.deviceBuilder import DeviceBuilder
from BAL.Exceptions.VersionError import VersionError, NEED_TO_UPDATE
from BAL.Handlers.incomingDataHandler import IncomingDataHandler
from BAL.Handlers.incomingHandler import IncomingHandler, MOTOR_RES, CLOSE_DIFF_RES, URF_RES, SWITCH_RES, IMU_RES, \
    GPS_RES, \
    PPM_RES, BAT_RES, IMU_CLIB_RES
from BAL.Handlers.incomingMsgHandler import IncomingMsgHandler
from BAL.Handlers.serialWriteHandler import SerialWriteHandler, HEADER_START, HEADER_DEBUG, KEEP_ALIVE_HEADER
from BAL.Header.Response.IMUPublishResponse import IMUPublishResponse
from BAL.Header.Response.ServoPublishResponse import ServoPublishResponse
from BAL.Header.Response.URFPublishResponse import URFPublishResponse
from BAL.Header.Response.VersionResponds import VersionResponds
from BAL.Header.Response.batteryPublishResponse import BatteryPublishResponse
from BAL.Header.Response.closeDiffPublishResponse import CloseDiffPublishRepose
from BAL.Header.Response.closeLoopPublishResponse import CloseLoopPublishResponse
from BAL.Header.Response.gpsPublishResponse import GPSPublishResponse
from BAL.Header.Response.imuCalibResponse import ImuCalibResponse
from BAL.Header.Response.ppmPublishResponse import PPMPublishResponse
from BAL.Header.Response.switchResponse import SwitchResponse

from BAL.ServerPkg.server import Server

__author__ = 'tom1231'
import roslib;

roslib.load_manifest('ric_board')
import rospy
import subprocess
import shlex
from BAL.Header.Response.ConnectionResponse import ConnectionResponse, RES_ID
from BAL.Header.Requests.ConnectionRequest import ConnectionRequest
from BAL.RiCParam.RiCParam import RiCParam
from serial import Serial, SerialException
from threading import Thread

CON_REQ = 1

SERVO_RES = 102
STATUS_RES = 100

INFO = 0
ERROR = 1
WARRNING = 2

VERSION = 7.0

WD_TIMEOUT = 5000


class Program:
    def __init__(self):
        try:

            self._toQuit = False

            rospy.init_node('RiCTraffic')
            params = RiCParam()
            ser = Serial('/dev/%s' % params.getConPort())
            ser.flushInput()
            ser.flushOutput()
            incomingHandler = IncomingHandler()
            input = ser
            output = SerialWriteHandler(ser, incomingHandler, input)
            data = []
            toPrint = ''
            input.baudrate = 9600
            incomingLength = 0
            headerId = 0
            devBuilder = DeviceBuilder(params, output, input, incomingHandler)
            gotHeaderStart = False
            gotHeaderDebug = False
            msgHandler = None
            server = None

            rospy.loginfo("Current version: %.2f" % VERSION)
            is_wd_active = False
            try:
                self.waitForConnection(output)
                if self.checkVer(input):
                    input.timeout = None
                    rospy.loginfo("Configuring devices...")
                    devBuilder.createServos()
                    devBuilder.createCLMotors()
                    devBuilder.createDiff()
                    devBuilder.createURF()
                    devBuilder.createSwitchs()
                    devBuilder.createPPM()
                    devBuilder.createIMU()
                    devBuilder.createRelays()
                    devBuilder.createGPS()
                    devBuilder.createOpenLoopMotors()
                    devBuilder.createBattery()
                    devBuilder.createOpenDiff()
                    devBuilder.createDiffFour()
                    devBuilder.createEmergencySwitch()
                    devs = devBuilder.getDevs()
                    devBuilder.sendFinishBuilding()
                    input.timeout = None
                    rospy.loginfo("Done, RiC Board is ready.")
                    msgHandler = IncomingMsgHandler(devs, output)
                    server = Server(devs, params)
                    Thread(target=self.checkForSubscribers, args=(devs,)).start()
                    Thread(target=msgHandler.run, args=()).start()
                    wd_keep_alive = int(round(time.time() * 1000))
                    while not rospy.is_shutdown() and not is_wd_active:
                        if gotHeaderStart:
                            if len(data) < 1:
                                data.append(input.read())
                                incomingLength, headerId = incomingHandler.getIncomingHeaderSizeAndId(data)
                            elif incomingLength >= 1:
                                for i in range(1, incomingLength):
                                    data.append(input.read())
                                msg = self.genData(data, headerId)
                                if msg is not None and msg.getId() != CON_REQ:
                                    msgHandler.addMsg(msg)
                                elif msg.getId() == CON_REQ and not msg.toConnect():
                                    subprocess.Popen(shlex.split("pkill -f RiCTraffic"))
                                    rospy.logerr("Emergency button is activated.")
                                    break
                                data = []
                                gotHeaderStart = False
                            else:
                                data = []
                                gotHeaderStart = False
                        elif gotHeaderDebug:
                            size = ord(input.read())

                            for i in xrange(size):
                                toPrint += input.read()

                            code = ord(input.read())

                            if code == INFO:
                                rospy.loginfo(toPrint)
                            elif code == ERROR:
                                rospy.logerr(toPrint)
                            elif code == WARRNING:
                                rospy.logwarn(toPrint)

                            toPrint = ''
                            gotHeaderDebug = False
                        elif input.inWaiting() > 0:
                            checkHead = ord(input.read())
                            if checkHead == HEADER_START:
                                gotHeaderStart = True
                            elif checkHead == HEADER_DEBUG:
                                gotHeaderDebug = True
                            elif checkHead == KEEP_ALIVE_HEADER:
                                wd_keep_alive = int(round(time.time() * 1000))
                        is_wd_active = (int(round(time.time() * 1000)) - wd_keep_alive) > WD_TIMEOUT
                else:
                    raise VersionError(NEED_TO_UPDATE)
                if is_wd_active:
                    rospy.logerr(
                        "RiCBoard isn't responding.\nThe Following things can make this happen:"
                        "\n1) If accidentally the manual driving is turn on, If so turn it off the relaunch the RiCBoard"
                        "\n2) If accidentally the RiCTakeovver gui is turn on,If so turn it off the relaunch the RiCBoard"
                        "\n3) The RiCBoard is stuck, If so please power off the robot and start it again. And contact RobotICan support by this email: tom@robotican.net")
            except KeyboardInterrupt:
                pass

            except VersionError:
                rospy.logerr("Can't load RiCBoard because the version don't mach please update the firmware.")


            finally:
                if not is_wd_active:
                    con = ConnectionResponse(False)
                    output.writeAndWaitForAck(con.dataTosend(), RES_ID)
                ser.close()
                if msgHandler != None: msgHandler.close()
                self._toQuit = True

        except SerialException:
            rospy.logerr("Can't find RiCBoard, please check if its connected to the computer.")

    def checkForSubscribers(self, devs):
        while not self._toQuit:
            if len(devs['servos']) > 0:
                for servo in devs['servos']:
                    servo.checkForSubscribers()
            if len(devs['motorsCl']) > 0:
                for motor in devs['motorsCl']:
                    motor.checkForSubscribers()
            if len(devs['urf']) > 0:
                for urf in devs['urf']:
                    urf.checkForSubscribers()
            if len(devs['switch']) > 0:
                for switch in devs['switch']:
                    switch.checkForSubscribers()
            if len(devs['diff']) > 0:
                devs['diff'][0].checkForSubscribers()
            if len(devs['imu']) > 0:
                devs['imu'][0].checkForSubscribers()
            if len(devs['gps']) > 0:
                devs['gps'][0].checkForSubscribers()
            if len(devs['ppm']) > 0:
                devs['ppm'][0].checkForSubscribers()
            if len(devs['battery']) > 0:
                devs['battery'][0].checkForSubscribers()

    def genData(self, data, headerId):
        result = None
        if headerId == CON_REQ: result = ConnectionRequest()
        if headerId == SERVO_RES: result = ServoPublishResponse()
        if headerId == MOTOR_RES: result = CloseLoopPublishResponse()
        if headerId == CLOSE_DIFF_RES: result = CloseDiffPublishRepose()
        if headerId == URF_RES: result = URFPublishResponse()
        if headerId == SWITCH_RES: result = SwitchResponse()
        if headerId == IMU_RES: result = IMUPublishResponse()
        if headerId == GPS_RES: result = GPSPublishResponse()
        if headerId == PPM_RES: result = PPMPublishResponse()
        if headerId == BAT_RES: result = BatteryPublishResponse()
        if headerId == IMU_CLIB_RES: result = ImuCalibResponse()

        if result is not None: result.buildRequest(data)
        return result

    def waitForConnection(self, output):
        output.writeAndWaitForAck(ConnectionResponse(True).dataTosend(), RES_ID)

    def checkVer(self, input):
        data = []
        gotHeaderStart = False
        verInfo = VersionResponds()
        input.setTimeout(1)
        countUntilTimeout = 0
        while not rospy.is_shutdown() and countUntilTimeout < 3:
            try:
                if gotHeaderStart:
                    for i in range(1, 11):
                        data.append(input.read())
                    verInfo.buildRequest(data)

                    if verInfo.checkPackage() and abs(verInfo.getVersion() - VERSION) < 1:
                        if verInfo.getVersion() < VERSION:
                            rospy.logwarn(
                                "RiCBord has a firmware %.2f please update the firmware for better performers" % (
                                    verInfo.getVersion()))
                        elif verInfo.getVersion() > VERSION:
                            rospy.logwarn(
                                "RiCBord has a firmware %.2f please update your package for better performers" % (
                                    verInfo.getVersion()))
                        return True
                    else:
                        return False

                elif ord(input.read()) == HEADER_START:
                    gotHeaderStart = True

            except TypeError:
                pass
            finally:
                countUntilTimeout += 1
        return False
