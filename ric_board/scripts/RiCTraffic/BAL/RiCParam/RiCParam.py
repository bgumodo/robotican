__author__ = 'tom1231'
import rospy


class RiCParam:
    def getServoNum(self):
        return int(rospy.get_param('servoNum', '0'))

    def getServoName(self, servoNum):
        return rospy.get_param('servo%d/name' % servoNum, '')

    def getServoPublishHz(self, servoNum):
        return int(rospy.get_param('servo%d/publishHz' % servoNum, '0'))

    def getServoPort(self, servoNum):
        return int(rospy.get_param('servo%d/port' % servoNum, '0'))

    def getServoMin(self, servoNum):
        return float(rospy.get_param('servo%d/min' % servoNum, '0'))

    def getServoMax(self, servoNum):
        return float(rospy.get_param('servo%d/max' % servoNum, '0'))

    def getServoInitMove(self, servoNum):
        return float(rospy.get_param('servo%d/initMove' % servoNum, '0'))

    def getServoAParameter(self, servoNum):
        return float(rospy.get_param('servo%d/a' % servoNum, '0'))

    def getServoBParameter(self, servoNum):
        return float(rospy.get_param('servo%d/b' % servoNum, '0'))

    def getServoData(self, servoNum):
        data = dict()
        data['devId'] = servoNum
        data['name'] = self.getServoName(servoNum)
        data['pubHz'] = self.getServoPublishHz(servoNum)
        data['port'] = self.getServoPort(servoNum)
        data['min'] = self.getServoMin(servoNum)
        data['max'] = self.getServoMax(servoNum)
        data['initMove'] = self.getServoInitMove(servoNum)
        data['a'] = self.getServoAParameter(servoNum)
        data['b'] = self.getServoBParameter(servoNum)
        return data

    def getCloseLoopMotorSize(self):
        return int(rospy.get_param('closeLoopNum', '0'))

    def getCloseLoopMotorName(self, motorNum):
        return rospy.get_param('closeLoop%d/name' % motorNum, '')

    def getCloseLoopMotorPubHz(self, motorNum):
        return int(rospy.get_param('closeLoop%d/publishHz' % motorNum, '0'))

    def getCloseLoopMotorLPFAlpha(self, motorNum):
        return float(rospy.get_param('closeLoop%d/LPFAlpha' % motorNum, '0'))

    def getCloseLoopMotorLPFHz(self, motorNum):
        return int(rospy.get_param('closeLoop%d/LPFHz' % motorNum, '0'))

    def getCloseLoopMotorDriverAddress(self, motorNum):
        return int(rospy.get_param('closeLoop%d/driverAddress' % motorNum, '0'))

    def getCloseLoopMotorChannel(self, motorNum):
        return int(rospy.get_param('closeLoop%d/channel' % motorNum, '0'))

    def getCloseLoopMotorEncoderPort(self, motorNum):
        return int(rospy.get_param('closeLoop%d/port' % motorNum, '0'))

    def getCloseLoopMotorPIDHz(self, motorNum):
        return int(rospy.get_param('closeLoop%d/PIDHz' % motorNum, '0'))

    def getCloseLoopMotorKp(self, motorNum):
        return float(rospy.get_param('closeLoop%d/kP' % motorNum, '0'))

    def getCloseLoopMotorKi(self, motorNum):
        return float(rospy.get_param('closeLoop%d/kI' % motorNum, '0'))

    def getCloseLoopMotorKd(self, motorNum):
        return float(rospy.get_param('closeLoop%d/kD' % motorNum, '0'))

    def getCloseLoopMotorMaxSpeed(self, motorNum):
        return float(rospy.get_param('closeLoop%d/maxSpeed' % motorNum, '0'))

    def getCloseLoopMotorCpr(self, motorNum):
        return int(rospy.get_param('closeLoop%d/cpr' % motorNum, '0'))

    def getCloseLoopMotorTimeout(self, motorNum):
        return int(rospy.get_param('closeLoop%d/timeout' % motorNum, '0'))

    def getCloseLoopMotorType(self, motorNum):
        return int(rospy.get_param('closeLoop%d/motorType' % motorNum, '0'))

    def getCloseLoopMotorDirection(self, motorNum):
        return int(rospy.get_param('closeLoop%d/direction' % motorNum, '0'))

    def getCloseLoopMotorDirectionEncoder(self, motorNum):
        return int(rospy.get_param('closeLoop%d/directionE' % motorNum, '0'))

    def getCloseLoopMotorIntegralLimit(self, motorNum):
        return float(rospy.get_param('closeLoop%d/limit' % motorNum, '0'))

    def getCloseLoopMotorEncoderType(self, motorNum):
        return int(rospy.get_param('closeLoop%d/encoderType' % motorNum, '0'))

    def getCloseLoopMotorPort2(self, motorNum):
        return int(rospy.get_param('closeLoop%d/port2' % motorNum, '0'))

    def isInitCloseDiff(self):
        return int(rospy.get_param('DIFF_INIT', '0')) == 1

    def isInitOpenDiff(self):
        return int(rospy.get_param('DIFF_INIT_OP', '0')) == 1

    def getCloseDiffPubHz(self):
        return int(rospy.get_param('Diff/publishHz', '0'))

    def getCloseDiffName(self):
        return rospy.get_param('Diff/name', '')

    def getCloseDiffRWheel(self):
        return float(rospy.get_param('Diff/rWheel', '0'))

    def getCloseDiffWidth(self):
        return float(rospy.get_param('Diff/width', '0'))

    def getCloseDiffBaseLink(self):
        return rospy.get_param('Diff/baseLink', '')

    def getCloseDiffOdom(self):
        return rospy.get_param('Diff/odom', '')

    def getCloseDiffSlip(self):
        return float(rospy.get_param('Diff/slip', '0'))

    def getCloseDiffMotorL(self):
        return int(rospy.get_param('Diff/motorL', '0'))

    def getCloseDiffMotorR(self):
        return int(rospy.get_param('Diff/motorR', '0'))

    def getCloseDiffMaxAng(self):
        return float(rospy.get_param('Diff/maxAng', '0'))

    def getCloseDiffMaxLin(self):
        return float(rospy.get_param('Diff/maxLin', '0'))

    def getURFNum(self):
        return int(rospy.get_param('URFNum', '0'))

    def getURFPubHz(self, urfNum):
        return int(rospy.get_param('URF%d/publishHz' % urfNum, '0'))

    def getURFName(self, urfNum):
        return rospy.get_param('URF%d/name' % urfNum, '')

    def getURFFrameId(self, urfNum):
        return rospy.get_param('URF%d/frameId' % urfNum, '')

    def getURFType(self, urfNum):
        return int(rospy.get_param('URF%d/type' % urfNum, '0'))

    def getURFPort(self, urfNum):
        return int(rospy.get_param('URF%d/port' % urfNum, '0'))

    def getSwitchSize(self):
        return int(rospy.get_param('switchNum', '0'))

    def getSwitchName(self, switchId):
        return rospy.get_param('switch%d/name' % switchId, '')

    def getSwitchPort(self, switchId):
        return int(rospy.get_param('switch%d/port' % switchId, '0'))

    def getSwitchPubHz(self, switchId):
        return int(rospy.get_param('switch%d/publishHz' % switchId, '0'))

    def isImuInit(self):
        return int(rospy.get_param('IMU_INIT', '0')) == 1

    def getIMUPubHz(self):
        return int(rospy.get_param('IMU/publishHz', '0'))

    def getIMUName(self):
        return rospy.get_param('IMU/name', '')

    def getIMUFrameId(self):
        return rospy.get_param('IMU/frameId', '')

    def getIMUCamp(self):
        return float(rospy.get_param('IMU/camp', '0.0'))

    def getIMUFusionHz(self):
        return int(rospy.get_param('IMU/fusionHz', '0'))

    def getIMUOrientation(self):
        return float(rospy.get_param('IMU/orientation', '0.0'))

    def isIMUFuseGyro(self):
        return int(rospy.get_param('IMU/fuseGyro', '0')) == 1

    def getRelaysSize(self):
        return int(rospy.get_param('relayNum', '0'))

    def getRelayName(self, relayNum):
        return rospy.get_param('relay%d/name' % relayNum, '')

    def getRelayPort(self, relayNum):
        return int(rospy.get_param('relay%d/port' % relayNum, '0'))

    def isGpsInit(self):
        return int(rospy.get_param('GPS_INIT', '0')) == 1

    def getGpsPubHz(self):
        return int(rospy.get_param('GPS/publishHz', '0'))

    def getGpsName(self):
        return rospy.get_param('GPS/name', '')

    def getGpsFrameId(self):
        return rospy.get_param('GPS/frameId', '')

    def getGpsBaudrate(self):
        return int(rospy.get_param('GPS/baudrate', '0'))

    def isXbeeEnable(self):
        return int(rospy.get_param('Xbee_INIT', '0'))

    def isPPMInit(self):
        return int(rospy.get_param('PPM_INIT', '0')) == 1

    def getPPMPubHz(self):
        return int(rospy.get_param('PPM/publishHz', '0'))

    def getPPMName(self):
        return rospy.get_param('PPM/name', '')

    def getOpenLoopNum(self):
        return int(rospy.get_param('openLoopNum', '0'))

    def getOpenLoopName(self, motorNum):
        return rospy.get_param('openLoop%d/name' % motorNum, '')

    def getOpenLoopAddress(self, motorNum):
        return int(rospy.get_param('openLoop%d/address' % motorNum, '0'))

    def getOpenLoopChannel(self, motorNum):
        return int(rospy.get_param('openLoop%d/channel' % motorNum, '0'))

    def getOpenLoopTimeout(self, motorNum):
        return int(rospy.get_param('openLoop%d/timeout' % motorNum, '0'))

    def getOpenLoopMax(self, motorNum):
        return int(rospy.get_param('openLoop%d/max' % motorNum, '0'))

    def getOpenLoopDirection(self, motorNum):
        return int(rospy.get_param('openLoop%d/direction' % motorNum, '0'))

    def isBatteryInit(self):
        return int(rospy.get_param('BAT_INIT', '0')) == 1

    def getBatteryName(self):
        return rospy.get_param('Battery/name', '')

    def getBatteryPubHz(self):
        return int(rospy.get_param('Battery/pubHz', '0'))

    def getBatteryVoltageDividerRatio(self):
        return float(rospy.get_param('Battery/voltageDividerRatio', '0.0'))

    def isCloseDiffFourInit(self):
        return int(rospy.get_param('DIFF_CLOSE_FOUR', '0')) == 1

    def getCloseDiffMotorFL(self):
        return int(rospy.get_param('Diff/motorFL', 0))

    def getCloseDiffMotorFR(self):
        return int(rospy.get_param('Diff/motorFR', 0))

    def getCloseDiffMotorBL(self):
        return int(rospy.get_param('Diff/motorBL', 0))

    def getCloseDiffMotorBR(self):
        return int(rospy.get_param('Diff/motorBR', 0))

    def getConPort(self):
        return rospy.get_param('CON_PORT', 'RiCBoard')

    def getFileName(self):
        return rospy.get_param('FILE_NAME', '')

    def getEmergencyPin(self, index):
        return int(rospy.get_param('emergency_switch{0}/pin'.format(str(index)), 27))

    def getEmergencyState(self, index):
        return int(rospy.get_param('emergency_switch{0}/state'.format(str(index)), 1))

    def getEmergencyName(self, index):
        return rospy.get_param('emergency_switch{0}/name'.format(str(index)), 'None')

    def EmergencyCount(self):
        return int(rospy.get_param('emergencySwitchNum', 0))

    def getBatteryMin(self):
        return float(rospy.get_param('Battery/min', '0.0'))

    def getBatteryMax(self):
        return float(rospy.get_param('Battery/max', '0.0'))
