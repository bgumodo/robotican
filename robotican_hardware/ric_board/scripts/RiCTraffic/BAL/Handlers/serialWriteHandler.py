import struct
import rospy
from BAL.Handlers.incomingHandler import ACK_RES
from BAL.Header.Response.ackResponse import ACKResponse

__author__ = 'tom1231'
from threading import RLock
HEADER_START = 0xfe
HEADER_DEBUG = 0xfd
KEEP_ALIVE_HEADER = 0xf9

class SerialWriteHandler:
    def __init__(self, ser, incomingDataHandler, input):
        self._incomingDataHandler = incomingDataHandler
        self._input = input
        self._serial = ser
        self._lock = RLock()

    def write(self, data):
        self._lock.acquire()
        self._serial.write(chr(HEADER_START))
        self._serial.write(str(data))
        self._lock.release()

    def writeAndWaitForAck(self, data, idToAck):
        self._lock.acquire()
        resend = True
        while resend:
            self.write(data)
            ack = self.waitForACK()
            if ack != None and ack.getIdToAck() == idToAck and ack.getReqLen() == len(data):
                resend = False
        self._lock.release()

    def waitForACK(self):
        gotHeaderStart = False
        incomingLength = 0
        headerId = 0
        data = []
        timeoutCount = 3
        try:
            while timeoutCount > 0:
                if gotHeaderStart:
                    if len(data) < 1:
                        data.append(self._input.read())
                        incomingLength, headerId = self._incomingDataHandler.getIncomingHeaderSizeAndId(data)
                    elif incomingLength >= 1 and headerId == ACK_RES:
                        for i in range(1, incomingLength):
                            data.append(self._input.read())
                        ack = ACKResponse()
                        ack.buildRequest(data)
                        if ack.checkPackage():
                            return ack
                        data = []
                        timeoutCount -= 1
                        gotHeaderStart = False
                    else:
                        data = []
                        timeoutCount -= 1
                        gotHeaderStart = False
                elif ord(self._input.read()) == HEADER_START:
                    gotHeaderStart = True
        except TypeError:
            rospy.logerr('ACK have not been send ,retransmitting.......')
        return None