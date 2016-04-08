#!/usr/bin/env python
__author__ = 'tom1231'
import roslib; roslib.load_manifest('ric_board')
import rospy
from rospy import Subscriber
from rospy import Publisher
from ric_board.msg import PPM
import sys
from geometry_msgs.msg import Twist


class Program:
    def main(self):
        if len(sys.argv) >= 3:
            rospy.init_node("RiC_PPM")
            topicNamePmm = sys.argv[1]
            topicNameDiff = sys.argv[2]
            # print 'PPM Topic: ' + topicNamePmm + '\n' + 'Diff Topic: ' + topicNameDiff
            self.pub = Publisher(topicNameDiff, Twist,queue_size=1)
            Subscriber(topicNamePmm, PPM, self.callBack, queue_size=1)
            rospy.spin()

    def callBack(self, msg):

        leftAndRight = 0
        upAndDown = 0

        if 1450 < msg.channels[0] < 1550: leftAndRight = 0
        else: leftAndRight = 0.032 * (float(msg.channels[0])) - 48

        if 1450 < msg.channels[1] < 1550: upAndDown = 0
        else: upAndDown = 0.032 * (float(msg.channels[1])) - 48

        msgPub = Twist()
        msgPub.angular.z = leftAndRight
        msgPub.linear.x = -upAndDown

        self.pub.publish(msgPub)

if __name__ == '__main__':
    Program().main()
