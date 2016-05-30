//
// Created by tom on 10/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_SERVO_H
#define ROBOTICAN_HARDWARE_INTERFACE_SERVO_H

#include <ros/ros.h>
#include <robotican_hardware_interface/Protocol.h>
#include <robotican_hardware_interface/TransportLayer.h>
#include <robotican_hardware_interface/Device.h>

#define SERVO_EPSILON 0.001

namespace robotican_hardware {
    class Servo : public Device {
    private:
        byte _pin;
        float _a;
        float _b;
        float _max;
        float _min;
        JointInfo_t _jointInfo;
        float _lastCmd;
        bool _isChangeParam;

        bool checkIfLastCmdChange();

    public:
        Servo(byte id, TransportLayer *transportLayer, byte pin, float a, float b, float max, float min, float initPos);
        virtual void update(const DeviceMessage *deviceMessage);
        virtual void write();
        virtual void deviceAck(const DeviceAck *ack);
        JointInfo_t* getJointInfo();
        void setParam(float a, float b, float max, float min);
        virtual void buildDevice();


    };

}



#endif //ROBOTICAN_HARDWARE_INTERFACE_SERVO_H
