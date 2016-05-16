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
    public:
        OpenLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        JointInfo_t* getJointInfo();

    protected:

        virtual void buildDevice();

    };

    struct CloseMotorParams {
        byte encoderPinA;
        byte encoderPinB;
        uint16_t LPFHz;
        uint16_t PIDHz;
        uint16_t CPR;
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

    class CloseLoopMotor : public RiCMotor {
    private:
        CloseMotorParams _params;
        JointInfo_t _jointInfo;
    public:
        CloseLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                               CloseMotorParams closeMotorParam);

        JointInfo_t* getJointInfo();

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

    protected:
        virtual void buildDevice();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_MOTOR_H
