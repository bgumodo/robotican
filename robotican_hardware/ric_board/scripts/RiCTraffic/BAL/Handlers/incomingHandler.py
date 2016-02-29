import struct

__author__ = 'tom1231'

CON_REQ = 1
IMU_REQ = 11
RELAY_REQ = 12
OPEN_LOOP_REQ = 15
RIG_REQ = 19
PSET_REQ = 20

SERVO_RES = 102
ACK_RES = 105
MOTOR_RES = 106
CLOSE_DIFF_RES = 108
URF_RES = 109
SWITCH_RES = 110
IMU_RES = 111
GPS_RES = 113
PPM_RES = 114
BAT_RES = 116
VER_RES = 118
IMU_CLIB_RES = 121

class IncomingHandler:

    def getIncomingHeaderId(self, data):
        return struct.unpack('<B', bytearray(data))[0]

    def getIncomingHeaderSizeAndId(self, data):
        id = self.getIncomingHeaderId(data)
        if id == CON_REQ: return 7, id
        if id == SERVO_RES: return 11, id
        if id == ACK_RES: return 8, id
        if id == MOTOR_RES: return 15, id
        if id == CLOSE_DIFF_RES: return 30, id
        if id == URF_RES: return 11, id
        if id == SWITCH_RES: return 8, id
        if id == IMU_RES: return 58, id
        if id == GPS_RES: return 23, id
        if id == PPM_RES: return 38, id
        if id == BAT_RES: return 10, id
        if id == IMU_CLIB_RES: return 18, id
        return 0, 0

