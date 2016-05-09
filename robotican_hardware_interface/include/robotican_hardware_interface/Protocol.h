//
// Created by tom on 04/05/16.
//

#ifndef RIC_BOARD_PROTOCOL_H
#define RIC_BOARD_PROTOCOL_H

#ifndef PC_SIDE
    #include <Arduino.h>
    enum DataType {
            Message = 0,
            Debug = 1,
            ConnectionState = 2,
            KeepAlive = 3,

    };
    enum DebugLevel {
        Info = 0,
        Warn = 1,
        Error = 2,
        Fatal = 3,
    };
    enum ConnectEnum {
        Connected = 1,
        NotReady = 2,
        AlreadyConnected = 3,
        Disconnected = 4,
        AlreadyDisconnected = 5,
    };

    enum KeepAliveState {
        Ok = 0,
        FatalError = 1,
        NeedToRestart = 2,
    };

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
            SwitchFeedBack = 7,
            UltrasonicFeedback = 8,
            RelySetState = 9,
            GpsFeedback = 10,
            ImuFeedback = 11,
            BatteryFeedback = 12,
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
};

#endif //RIC_BOARD_PROTOCOL_H
