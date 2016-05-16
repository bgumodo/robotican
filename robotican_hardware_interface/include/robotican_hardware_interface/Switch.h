//
// Created by tom on 16/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_SWITCH_H
#define ROBOTICAN_HARDWARE_INTERFACE_SWITCH_H

#include <std_msgs/Bool.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    class Switch : public Device {
    private:
        byte _pin;
        std::string _topicName;
        ros::Publisher _switchState;

    public:
        Switch(byte id, TransportLayer *transportLayer, byte pin, std::string topicName);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        virtual void deviceAck(const DeviceAck *ack);
    protected:
        virtual void buildDevice();

    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_SWITCH_H
