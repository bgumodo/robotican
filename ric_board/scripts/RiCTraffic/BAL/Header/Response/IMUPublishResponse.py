__author__ = 'tom1231'
import struct
from BAL.Header.RiCHeader import RiCHeader
from geometry_msgs.msg import Vector3, Quaternion

V_X_LEN = 10
V_Y_LEN = 14
V_Z_LEN = 18

A_X_LEN = 22
A_Y_LEN = 26
A_Z_LEN = 30

M_X_LEN = 34
M_Y_LEN = 38
M_Z_LEN = 42

O_X_LEN = 46
O_Y_LEN = 50
O_Z_LEN = 54
O_W_LEN = 58


# ROLL_LEN = 64
# PITCH_LEN = 68
# YAW_LEN = 72
# TEMP_LEN = 76

class IMUPublishResponse(RiCHeader):
    def __init__(self):
        RiCHeader.__init__(self)
        self._velocityX = 0.0
        self._velocityY = 0.0
        self._velocityZ = 0.0

        self._accelerationX = 0.0
        self._accelerationY = 0.0
        self._accelerationZ = 0.0

        self._magnetometerX = 0.0
        self._magnetometerY = 0.0
        self._magnetometerZ = 0.0

        self._orientationX = 0.0
        self._orientationY = 0.0
        self._orientationZ = 0.0
        self._orientationW = 0.0

        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._temperature = 0.0

    def getImuMsgId(self):
        return 2

    def getVelocity(self):
        vec = Vector3()
        vec.y = -1 * (self._velocityX * 0.0174532925)  # deg/s -> rad/s
        vec.x = -1 * (self._velocityY * 0.0174532925)  # deg/s -> rad/s
        vec.z = -1 * (self._velocityZ * 0.0174532925)  # deg/s -> rad/s
        return vec

    def getAcceleration(self):
        acc = Vector3()
        acc.y = self._accelerationX * 9.80665  # g -> m/s^2
        acc.x = self._accelerationY * 9.80665  # g -> m/s^2
        acc.z = self._accelerationZ * 9.80665  # g -> m/s^2
        return acc

    def getMagnetometer(self):
        mag = Vector3()
        mag.x = self._magnetometerX * 1.0e-7  # milligauss -> Teslas
        mag.y = self._magnetometerY * 1.0e-7  # milligauss -> Teslas
        mag.z = self._magnetometerZ * 1.0e-7  # milligauss -> Teslas
        return mag

    def getOrientation(self):
        ort = Quaternion()
        ort.x = self._orientationX
        ort.y = self._orientationY
        ort.z = self._orientationZ
        ort.w = self._orientationW
        return ort

    def getRoll(self):
        return self._roll

    def getPitch(self):
        return self._pitch

    def getYaw(self):
        return self._yaw

    def getTemperature(self):
        return self._temperature

    def buildRequest(self, data):
        RiCHeader.buildRequest(self, data)
        bytes = bytearray()
        while self.index < V_X_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._velocityX = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < V_Y_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._velocityY = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < V_Z_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._velocityZ = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < A_X_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._accelerationX = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < A_Y_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._accelerationY = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < A_Z_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._accelerationZ = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < M_X_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._magnetometerX = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < M_Y_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._magnetometerY = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < M_Z_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._magnetometerZ = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < O_X_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._orientationX = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < O_Y_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._orientationY = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < O_Z_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._orientationZ = struct.unpack('<f', bytes)[0]
        bytes = bytearray()
        while self.index < O_W_LEN:
            bytes.append(data[self.index])
            self.index += 1
        self._orientationW = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < ROLL_LEN:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._roll = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < PITCH_LEN:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._pitch = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < YAW_LEN:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._yaw = struct.unpack('<f', bytes)[0]
        # bytes = bytearray()
        # while self.index < TEMP_LEN:
        #     bytes.append(data[self.index])
        #     self.index += 1
        # self._temperature = struct.unpack('<f', bytes)[0]
