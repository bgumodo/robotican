//
// Created by tom on 04/05/16.
//

#ifndef RIC_BOARD_PROTOCOL_H
#define RIC_BOARD_PROTOCOL_H

#ifndef PC_SIDE
    #include <Arduino.h>

     namespace DeviceMessageType {
        enum DeviceMessageType {
            BuildDevice = 0,
            Ack = 1,
            MotorSetPointMsg = 2,
            MotorFeedback = 3,
            MotorSetPid = 4,
            ServoFeedback = 5,
            ServoSetPoint = 6,
            SwitchFeedBack = 7,
            UltrasonicFeedback = 8,
            RelySetState = 9,
            GpsFeedback = 10,
            ImuFeedback = 11,
            BatteryFeedback = 12,
        };
    }

    namespace DeviceType {
        enum DevieType {
            Battery = 0,
            MotorCloseLoop = 1,
            MotorOpenLoop = 2,
            Imu = 3,
            Gps = 4,
            Servo = 5,
            Ultrasonic = 6,
            Switch = 7,
            Realy = 8,

        };
    namespace CloseMotorType {
        enum CloseMotorType {
            CloseLoopWithEncoder = 0,
        };
    }

    namespace CloseMotorMode {
        enum CloseMotorMode {
            Speed = 0,
            POSITION = 1,
        };
    }

    namespace DataType {
        enum DataType {
            DeviceMessage = 0,
            Debug = 1,
            ConnectionState = 2,
            KeepAlive = 3,

        };
    }
    namespace DebugLevel {
        enum DebugLevel {
            Info = 0,
            Warn = 1,
            Error = 2,
            Fatal = 3,
        };
    }
    namespace ConnectEnum {
        enum ConnectEnum {
            Connected = 1,
            NotReady = 2,
            AlreadyConnected = 3,
            Disconnected = 4,
            AlreadyDisconnected = 5,
        };
    }

    namespace KeepAliveState {
        enum KeepAliveState {
            Ok = 0,
            FatalError = 1,
            NeedToRestart = 2,
        };
    }

#endif

#ifdef PC_SIDE
    #include <stdint.h>
    typedef uint8_t byte;

    namespace DeviceMessageType {
        enum DeviceMessageType {
            BuildDevice = 0,
            Ack = 1,
            MotorSetPointMsg = 2,
            MotorFeedback = 3,
            MotorSetPid = 4,
            ServoFeedback = 5,
            ServoSetPoint = 6,
            SwitchFeedback = 7,
            UltrasonicFeedback = 8,
            RelySetState = 9,
            GpsFeedback = 10,
            ImuFeedback = 11,
            BatteryFeedback = 12,
        };
    }

    namespace DeviceType {
        enum DeviceType {
            Battery = 0,
            MotorCloseLoop = 1,
            MotorOpenLoop = 2,
            Imu = 3,
            Gps = 4,
            Servo = 5,
            Ultrasonic = 6,
            Switch = 7,
            Relay = 8,

        };
    }
    namespace CloseMotorType {
            enum CloseMotorType {
                CloseLoopWithEncoder = 0,
            };
    }

    namespace CloseMotorMode {
        enum CloseMotorMode {
            Speed = 0,
            POSITION = 1,
        };
    }


    namespace DataType {
        enum DataType {
            DeviceMessage = 0,
            Debug = 1,
            ConnectionState = 2,
            KeepAlive = 3,

        };
    }
    namespace DebugLevel {
        enum DebugLevel {
            Info = 0,
            Warn = 1,
            Error = 2,
            Fatal = 3,
        };
    }
    namespace ConnectEnum {
        enum ConnectEnum {
            Connected = 1,
            NotReady = 2,
            AlreadyConnected = 3,
            Disconnected = 4,
            AlreadyDisconnected = 5,
        };
    }

    namespace KeepAliveState {
        enum KeepAliveState {
            Ok = 0,
            FatalError = 1,
            NeedToRestart = 2,
        };
    }
#endif

#define HEADER_SIGNAL 0xFF


struct Header {
    byte length;
    byte dataType;
    uint16_t checkSum;
}__attribute__((__packed__));

struct DebugMsg : Header {
    DebugMsg() {
        dataType = DataType::Debug;
    }
    byte level;
    char message[128];
}__attribute__((__packed__));

struct ConnectState : Header{
    ConnectState() {
        dataType = DataType::ConnectionState;
    }
    byte state;
    byte version;
}__attribute__((__packed__));

struct KeepAliveMsg : Header {
    KeepAliveMsg() {
        dataType = DataType::KeepAlive;
    }

    byte state;
}__attribute__((__packed__));

struct DeviceMessage : Header {
    DeviceMessage() {
        dataType = DataType::DeviceMessage;
    }

    byte id;
    byte deviceMessageType;
}__attribute__((__packed__));


struct DeviceAck : DeviceMessage {
    DeviceAck() : DeviceMessage() {
        deviceMessageType = DeviceMessageType::Ack;
    }

    byte ackId;
}__attribute__((__packed__));

struct BuildDevice : DeviceMessage {
    BuildDevice() : DeviceMessage() {
        deviceMessageType = DeviceMessageType::BuildDevice;
    }
    byte deviceType;
}__attribute__((__packed__));

struct BuildBattery : BuildDevice {
    BuildBattery() {
        deviceType = DeviceType::Battery;
    }
    byte pin;
}__attribute__((__packed__));

struct BuildUltrasonic : BuildDevice {
    BuildUltrasonic() {
        deviceType = DeviceType::Ultrasonic;
    }
    byte pin;
}__attribute__((__packed__));

struct BuildGps : BuildDevice {
    BuildGps() {
        deviceType = DeviceType::Gps;
    }
    unsigned int baudrate;
}__attribute__((__packed__));

struct BuildImu : BuildDevice {
    BuildImu() {
        deviceType = DeviceType::Imu;
    }
    uint16_t fusionHz;
    bool enableGyro;
}__attribute__((__packed__));

struct BuildSwitch : BuildDevice {
    BuildSwitch() {
        deviceType = DeviceType::Switch;
    }
    byte pin;
};

struct BuildRelay : BuildDevice {
    BuildRelay() {
        deviceType = DeviceType::Relay;
    }
    byte pin;
};


struct BuildMotorCloseLoop : BuildDevice {
    BuildMotorCloseLoop() {
        deviceType = DeviceType::MotorCloseLoop;
    }
    byte motorAddress;
    byte motorMode;
    byte eSwitchPin;
    byte eSwitchType;
    byte motorType;
    uint16_t LPFHz;
    uint16_t PIDHz;
    uint16_t PPR;
    uint16_t timeout;
    int8_t motorDirection;
    int8_t encoderDirection;
    float LPFAlpha;
    float KP;
    float KI;
    float KD;
    float maxSpeed;
    float limit;
}__attribute__((__packed__));

struct BuildMotorCloseLoopWithEncoder : BuildMotorCloseLoop {
    BuildMotorCloseLoopWithEncoder() {
        motorType = CloseMotorType::CloseLoopWithEncoder;
    }
    byte encoderPinA;
    byte encoderPinB;
}__attribute__((__packed__));



struct BuildMotorOpenLoop : BuildDevice {
    BuildMotorOpenLoop() {
        deviceType = DeviceType::MotorOpenLoop;
    }
    byte motorAddress;
    byte eSwitchPin;
    byte eSwitchType;
    float maxSpeed;

}__attribute__((__packed__));

struct BuildServo : BuildDevice {
    BuildServo() {
        deviceType = DeviceType::Servo;
    }
    byte pin;
    float a;
    float b;
    float max;
    float min;
}__attribute__((__packed__));

struct ServoFeedback : DeviceMessage {
    ServoFeedback() {
        deviceMessageType = DeviceMessageType::ServoFeedback;
    }
    float pos;
}__attribute__((__packed__));

struct SwitchFeedback : DeviceMessage {
    SwitchFeedback() {
        deviceMessageType = DeviceMessageType::SwitchFeedback;
    }
    bool state;
}__attribute__((__packed__));

struct GpsFeedback : DeviceMessage {
    GpsFeedback() {
        deviceMessageType = DeviceMessageType::GpsFeedback;
    }
    float lat;
    float lng;
    float meters;
    int16_t HDOP;
    int16_t satellites;
    int8_t fix;
}__attribute__((__packed__));

struct BatteryFeedback : DeviceMessage {
    BatteryFeedback() {
        deviceMessageType = DeviceMessageType::BatteryFeedback;
    }
    uint16_t currentRead;
}__attribute__((__packed__));

struct ImuFeedback : DeviceMessage {
    ImuFeedback() {
        deviceMessageType = DeviceMessageType::ImuFeedback;
    }
    float velocityX;
    float velocityY;
    float velocityZ;

    float accelerationX;
    float accelerationY;
    float accelerationZ;

    float magnetometerX;
    float magnetometerY;
    float magnetometerZ;

    float orientationX;
    float orientationY;
    float orientationZ;
    float orientationW;

}__attribute__((__packed__));



struct UltrasonicFeedback : DeviceMessage {
    UltrasonicFeedback() {
        deviceMessageType = DeviceMessageType::UltrasonicFeedback;
    }
    uint16_t currentRead;
}__attribute__((__packed__));


struct MotorFeedback : DeviceMessage {
    MotorFeedback() {
        deviceMessageType = DeviceMessageType::MotorFeedback;
    }
    float rad;
    float rad_s;
}__attribute__((__packed__));

struct ServoSetPoint : DeviceMessage {
    ServoSetPoint() {
        deviceMessageType = DeviceMessageType::ServoSetPoint;
    }
    float pos;
}__attribute__((__packed__));

struct MotorSetPoint : DeviceMessage {
    MotorSetPoint() {
        deviceMessageType = DeviceMessageType::MotorSetPointMsg;
    }
    float point;
}__attribute__((__packed__));

struct RelaySetState : DeviceMessage {
    RelaySetState() {
        deviceMessageType = DeviceMessageType::RelySetState;
    }
    bool state;
}__attribute__((__packed__));


#endif //RIC_BOARD_PROTOCOL_H
