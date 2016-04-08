from threading import Thread
from BAL.Devices.RICOpenLoopMotor import OpenLoopMotor
from BAL.Devices.RiCBattery import RiCBattery
from BAL.Devices.RiCDiffCloseLoop import RiCDiffCloseLoop
from BAL.Devices.RiCGPS import RiCGPS
from BAL.Devices.RiCIMU import RiCIMU
from BAL.Devices.RiCOpenDiff import RiCOpenDiff
from BAL.Devices.RiCPPM import RiCPPM
from BAL.Devices.RiCRelay import RiCRelay
from BAL.Devices.RiCSwitch import RiCSwitch
from BAL.Devices.RiCURF import RiCURF
from BAL.Header.Response.BatteryParamResponse import BatteryParamResponse
from BAL.Header.Response.CloseLoopMotorTwoEncBuildResponse import CloseLoopMotorTwoEncBuildResponse
from BAL.Header.Response.IMUParamResponse import IMUParamResponse
from BAL.Header.Response.ParamBuildResponse import EngineCL, EngineCL2
from BAL.Header.Response.closeDiffFourParamResponse import CloseDIffFourParamResponse
from BAL.Header.Response.closeDiffParamResponse import CloseDiffParamResponse
from BAL.Header.Response.emergencySwitchParamResponse import EmergencySwitchParamResponse
from BAL.Header.Response.gpsParamResponse import GPSParamResponse
from BAL.Header.Response.openLoopMotorParamResponse import OpenLoopMotorParamResponse
from BAL.Header.Response.ppmParamResponse import PPMParamResponse
from BAL.Header.Response.relayParamResponse import RelayParamResponse
from BAL.Header.Response.switchParamResponse import SwitchParamResponse
from BAL.Header.Response.urfParamResponse import URFParamResponse

__author__ = 'tom1231'

import rospy
from BAL.Header.Requests.finishBuildingRequest import FinishBuildingRequest, ID_REQ
from BAL.Devices.RiCCloseLoopMotor import RiCCloseLoopMotor
from BAL.Devices.RiCServo import RiCServo
from BAL.Header.Response.BuildServoResponse import BuildServoResponse
from BAL.Header.Response.closeLoopMotorBuildResponse import CloseLoopMotorBuildResponse
from BAL.Header.Response.ackResponse import ACKResponse
from BAL.Handlers.incomingHandler import ACK_RES
from BAL.Handlers.serialWriteHandler import HEADER_START


class DeviceBuilder:
    def __init__(self, param, output, input, incomingDataHandle):
        self._param = param
        self._output = output
        self._input = input
        self._incomingDataHandler = incomingDataHandle
        self._allDevs = dict()
        self._allDevs['servos'] = []
        self._allDevs['motorsCl'] = []
        self._allDevs['motorsOl'] = []
        self._allDevs['urf'] = []
        self._allDevs['switch'] = []
        self._allDevs['diff'] = []
        self._allDevs['imu'] = []
        self._allDevs['relay'] = []
        self._allDevs['gps'] = []
        self._allDevs['ppm'] = []
        self._allDevs['battery'] = []

    def createServos(self):
        servoAmount = self._param.getServoNum()
        for servoNum in xrange(servoAmount):
            rospy.loginfo("Configuring servo: %s", self._param.getServoName(servoNum))
            self._allDevs['servos'].append(RiCServo(self._param, servoNum, self._output))
            servoTORic = BuildServoResponse(servoNum, self._param)
            self._output.writeAndWaitForAck(servoTORic.dataTosend(), servoNum)
            rospy.loginfo("Servo: %s is ready ", self._param.getServoName(servoNum))

    def createCLMotors(self):
        closeMotorsAmount = self._param.getCloseLoopMotorSize()
        for motorNum in xrange(closeMotorsAmount):
            rospy.loginfo("Configuring motor: %s", self._param.getCloseLoopMotorName(motorNum))
            motor = RiCCloseLoopMotor(motorNum, self._param, self._output)
            self._allDevs['motorsCl'].append(motor)
            if self._param.getCloseLoopMotorEncoderType(motorNum) == 1:
                toSend = CloseLoopMotorBuildResponse(motorNum, self._param, EngineCL)
            else:
                toSend = CloseLoopMotorTwoEncBuildResponse(motorNum, self._param, EngineCL2)
            self._output.writeAndWaitForAck(toSend.dataTosend(), motorNum)
            rospy.loginfo("Motor: %s is ready", self._param.getCloseLoopMotorName(motorNum))

    def createDiff(self):
        if self._param.isInitCloseDiff():
            rospy.loginfo("Configuring differential drive: %s", self._param.getCloseDiffName())
            diff = RiCDiffCloseLoop(self._param, self._output)
            self._allDevs['diff'].append(diff)
            toSend = CloseDiffParamResponse(0, self._param)
            self._output.writeAndWaitForAck(toSend.dataTosend(), 0)
            rospy.loginfo("Differential drive: %s is ready", self._param.getCloseDiffName())

    def createDiffFour(self):
        if self._param.isCloseDiffFourInit():
            rospy.loginfo("Configuring differential drive: %s", self._param.getCloseDiffName())
            diff = RiCDiffCloseLoop(self._param, self._output)
            self._allDevs['diff'].append(diff)
            toSend = CloseDIffFourParamResponse(0, self._param)
            self._output.writeAndWaitForAck(toSend.dataTosend(), 0)
            rospy.loginfo("Differential drive: %s is ready", self._param.getCloseDiffName())

    def createURF(self):
        URFAmount = self._param.getURFNum()
        for urfId in xrange(URFAmount):
            rospy.loginfo("Configuring URF: %s", self._param.getURFName(urfId))
            urf = RiCURF(urfId, self._param, self._output)
            self._allDevs['urf'].append(urf)
            self._output.writeAndWaitForAck(URFParamResponse(urf.getType(), urfId, self._param).dataTosend(), urfId)
            rospy.loginfo("URF: %s is ready", self._param.getURFName(urfId))

    def createSwitchs(self):
        switchAmount = self._param.getSwitchSize()
        for switchNum in xrange(switchAmount):
            rospy.loginfo("Configuring switch: %s", self._param.getSwitchName(switchNum))
            switch = RiCSwitch(switchNum, self._param, self._output)
            self._allDevs['switch'].append(switch)
            self._output.writeAndWaitForAck(SwitchParamResponse(switchNum, self._param).dataTosend(), switchNum)
            rospy.loginfo("Switch: %s is ready", self._param.getSwitchName(switchNum))

    def createIMU(self):
        if self._param.isImuInit():
            rospy.loginfo("Configuring IMU: %s", self._param.getIMUName())
            imu = RiCIMU(self._param, self._output)
            self._allDevs['imu'].append(imu)
            self._output.writeAndWaitForAck(IMUParamResponse(self._param).dataTosend(), 0)
            rospy.loginfo("IMU: %s is ready", self._param.getIMUName())

    def createRelays(self):
        relayAmount = self._param.getRelaysSize()
        for relayNum in xrange(relayAmount):
            rospy.loginfo("Configuring relay: %s", self._param.getRelayName(relayNum))
            relay = RiCRelay(self._param, relayNum, self._output)
            self._output.writeAndWaitForAck(RelayParamResponse(relayNum, self._param).dataTosend(), relayNum)
            self._allDevs['relay'].append(relay)
            rospy.loginfo("Relay: %s is ready", self._param.getRelayName(relayNum))

    def createGPS(self):
        if self._param.isGpsInit():
            rospy.loginfo("Configuring GPS: %s", self._param.getGpsName())
            gps = RiCGPS(self._param, self._output)
            self._output.writeAndWaitForAck(GPSParamResponse(self._param).dataTosend(), 0)
            self._allDevs['gps'].append(gps)
            rospy.loginfo("GPS: %s is ready", self._param.getGpsName())

    def createPPM(self):
        if self._param.isPPMInit():
            rospy.loginfo("Configuring PPM: %s", self._param.getPPMName())
            ppm = RiCPPM(self._param, self._output)
            self._output.writeAndWaitForAck(PPMParamResponse(self._param).dataTosend(), 0)
            self._allDevs['ppm'].append(ppm)
            rospy.loginfo("PPM: %s is ready", self._param.getPPMName())

    def createOpenLoopMotors(self):
        motorsAmout = self._param.getOpenLoopNum()
        for motorId in xrange(motorsAmout):
            rospy.loginfo("Configuring motor: %s", self._param.getOpenLoopName(motorId))
            motor = OpenLoopMotor(motorId, self._param, self._output)
            self._allDevs['motorsOl'].append(motor)
            self._output.writeAndWaitForAck(OpenLoopMotorParamResponse(motorId,self._param).dataTosend(), motorId)
            rospy.loginfo("Motor: %s is ready", self._param.getOpenLoopName(motorId))

    def createBattery(self):
        if self._param.isBatteryInit():
            rospy.loginfo("Configuring battery: %s", self._param.getBatteryName())
            battery = RiCBattery(self._param,self._output)
            self._allDevs['battery'].append(battery)
            self._output.writeAndWaitForAck(BatteryParamResponse(self._param).dataTosend(), 0)
            rospy.loginfo("Battery: %s is ready", self._param.getBatteryName())

    def createOpenDiff(self):
        if self._param.isInitOpenDiff():
            rospy.loginfo("Configuring differential drive: %s", self._param.getCloseDiffName())
            motorL = self._allDevs['motorsOl'][self._param.getCloseDiffMotorL()]
            motorR = self._allDevs['motorsOl'][self._param.getCloseDiffMotorR()]
            diff = RiCOpenDiff(self._param, motorL, motorR)
            self._allDevs['diff'].append(diff)
            rospy.loginfo("Differential drive: %s is ready", self._param.getCloseDiffName())

    def createEmergencySwitch(self):
        size = self._param.EmergencyCount()
        for i in xrange(size):
            rospy.loginfo("Configuring emergency switch: %s", self._param.getEmergencyName(i))
            self._output.writeAndWaitForAck(EmergencySwitchParamResponse(i, self._param).dataTosend(), i)
            rospy.loginfo("Emergency switch is ready: %s", self._param.getEmergencyName(i))

    def getDevs(self):
        return self._allDevs


    def sendFinishBuilding(self):
        self._output.writeAndWaitForAck(FinishBuildingRequest().dataTosend(), ID_REQ)


