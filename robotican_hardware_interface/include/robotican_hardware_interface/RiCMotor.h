//
// Created by tom on 15/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H
#define ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H

#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>


namespace robotican_hardware {
    class RiCMotor : public Device {
    private:
        byte _motorAddress;
        byte _eSwitchPin;
        byte _eSwitchType;
    public:
        RiCMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType);

        virtual void deviceAck(const DeviceAck *ack);

        virtual void update(const DeviceMessage *deviceMessage)=0;

        virtual void write()=0;

    protected:
        byte getESwitchPin();
        byte getESwitchType();
        byte getAddress();
        virtual void buildDevice()=0;
    };

    class OpenLoopMotor : public RiCMotor {
    private:
        JointInfo_t _jointInfo;
        float _maxSpeed;
    public:

        OpenLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin,
                              byte eSwitchType, float maxSpeed);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        JointInfo_t* getJointInfo();

    protected:

        virtual void buildDevice();
    };

    struct CloseMotorParams {
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
    };

    struct  CloseMotorWithEncoderParam : CloseMotorParams {
        byte encoderPinA;
        byte encoderPinB;
    };

    class CloseLoopMotor : public RiCMotor {
    private:
        JointInfo_t _jointInfo;
        CloseMotorType::CloseMotorType _motorType;
        CloseMotorMode::CloseMotorMode _mode;
    public:
        CloseLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                               CloseMotorType::CloseMotorType motorType, CloseMotorMode::CloseMotorMode mode);

        JointInfo_t* getJointInfo();

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();
    protected:
        CloseMotorType::CloseMotorType getCloseMotorType();
        CloseMotorMode::CloseMotorMode getMode();
        virtual void buildDevice() = 0;
    };

    class CloseLoopMotorWithEncoder : public CloseLoopMotor {
    private:
        CloseMotorWithEncoderParam _params;
    public:
        CloseLoopMotorWithEncoder(byte id, TransportLayer *transportLayer, byte motorAddress,
                                          byte eSwitchPin, byte eSwitchType,
                                          CloseMotorType::CloseMotorType motoryType,
                                          CloseMotorMode::CloseMotorMode mode,
                                          CloseMotorWithEncoderParam param);

    protected:
        virtual void buildDevice();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H
