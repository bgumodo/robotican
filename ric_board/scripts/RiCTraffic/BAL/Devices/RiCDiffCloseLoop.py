import re
from threading import Thread
from nav_msgs.msg import Odometry
import rospy
import rostopic
from BAL.Header.Requests.PublishRequest import PublishRequest
from BAL.Header.Requests.closeDiffRequest import CloseDiffRequest
from BAL.Header.Requests.closeDiffSetOdomRequest import CloseDiffSetOdomRequest
from BAL.Header.Response.ParamBuildResponse import DiffDriverCL

__author__ = 'tom1231'
from BAL.Interfaces.Device import Device
from rospy import Subscriber, Service, Publisher
from ric_board.srv._set_odom import set_odom, set_odomResponse
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
import math


class RiCDiffCloseLoop(Device):
    def __init__(self, param, output):
        Device.__init__(self, param.getCloseDiffName(), output)
        self._baseLink = param.getCloseDiffBaseLink()
        self._odom = param.getCloseDiffOdom()
        self._maxAng = param.getCloseDiffMaxAng()
        self._maxLin = param.getCloseDiffMaxLin()
        self._pub = Publisher("%s/odometry" % self._name, Odometry, queue_size=param.getCloseDiffPubHz())
        self._broadCase = TransformBroadcaster()
        Subscriber('%s/command' % self._name, Twist, self.diffServiceCallback, queue_size=1)
        Service('%s/setOdometry' % self._name, set_odom, self.setOdom)
        self._haveRightToPublish = False
        self._prevOdom = None
        self._firstTimePub = True

    def getType(self):
        return DiffDriverCL

    def diffServiceCallback(self, msg):
        Thread(target=self.sendMsg, args=(msg,)).start()

    def sendMsg(self, msg):
        if msg.angular.z > self._maxAng:
            msg.angular.z = self._maxAng
        elif msg.angular.z < -self._maxAng:
            msg.angular.z = -self._maxAng

        if msg.linear.x > self._maxLin:
            msg.linear.x = self._maxLin
        elif msg.linear.x < -self._maxLin:
            msg.linear.x = -self._maxLin
        # print msg.angular.z, msg.linear.x
        self._output.write(CloseDiffRequest(msg.angular.z, msg.linear.x).dataTosend())

    def setOdom(self, req):
        self._output.write(CloseDiffSetOdomRequest(req.x, req.y, req.theta).dataTosend())
        return set_odomResponse(True)

    def publish(self, data):
        q = Quaternion()
        q.x = 0
        q.y = 0
        q.z = data[6]
        q.w = data[7]
        odomMsg = Odometry()
        odomMsg.header.frame_id = self._odom
        odomMsg.header.stamp = rospy.get_rostime()
        odomMsg.child_frame_id = self._baseLink
        odomMsg.pose.pose.position.x = data[0]
        odomMsg.pose.pose.position.y = data[1]
        odomMsg.pose.pose.position.z = 0
        odomMsg.pose.pose.orientation = q
        if self._firstTimePub:
            self._prevOdom = odomMsg
            self._firstTimePub = False
            return

        velocity = Twist()

        deltaTime = odomMsg.header.stamp.to_sec() - self._prevOdom.header.stamp.to_sec()
        yaw, pitch, roll = euler_from_quaternion(
            [odomMsg.pose.pose.orientation.w, odomMsg.pose.pose.orientation.x, odomMsg.pose.pose.orientation.y,
             odomMsg.pose.pose.orientation.z])
        prevYaw, prevPitch, prevRollprevYaw = euler_from_quaternion(
            [self._prevOdom.pose.pose.orientation.w, self._prevOdom.pose.pose.orientation.x,
             self._prevOdom.pose.pose.orientation.y, self._prevOdom.pose.pose.orientation.z])
        if deltaTime > 0:
            velocity.linear.x = (data[8] / deltaTime)

        deltaYaw = yaw - prevYaw

        # rospy.loginfo("yaw: %f\t\tpevYaw: %f\t\tdeltaYaw: %f" % (yaw,prevYaw,deltaYaw))

        if deltaYaw > math.pi: deltaYaw -= 2 * math.pi
        elif deltaYaw < -math.pi: deltaYaw += 2 * math.pi

        if deltaTime > 0:
            velocity.angular.z = -(deltaYaw / deltaTime)

        # rospy.loginfo("deltaYaw after check: %f\t\t angular: %f" % (deltaYaw, velocity.angular.z))

        odomMsg.twist.twist = velocity

        self._prevOdom = odomMsg

        traMsg = TransformStamped()
        traMsg.header.frame_id = self._odom
        traMsg.header.stamp = rospy.get_rostime()
        traMsg.child_frame_id = self._baseLink
        traMsg.transform.translation.x = data[0]
        traMsg.transform.translation.y = data[1]
        traMsg.transform.translation.z = 0
        traMsg.transform.rotation = q

        self._pub.publish(odomMsg)
        self._broadCase.sendTransformMessage(traMsg)

    def checkForSubscribers(self):
        try:
            subCheck = re.search('Subscribers:.*', rostopic.get_info_text(self._pub.name)).group(0).split(': ')[1]
            subTfCheck = re.search('Subscribers:.*', rostopic.get_info_text('/tf')).group(0).split(': ')[1]

            if not self._haveRightToPublish and (subCheck == '' or subTfCheck == ''):
                self._output.write(PublishRequest(DiffDriverCL, 0, True).dataTosend())
                self._haveRightToPublish = True

            elif self._haveRightToPublish and (subCheck == 'None' and subTfCheck == 'None'):
                self._output.write(PublishRequest(DiffDriverCL, 0, False).dataTosend())
                self._haveRightToPublish = False
                self._firstTimePub = True
        except:
            pass
