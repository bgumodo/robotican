//
// Created by tom on 16/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_RELAY_H
#define ROBOTICAN_HARDWARE_INTERFACE_RELAY_H

#include <ric_board/Relay.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    class Relay : public Device {
    private:
        byte _pin;
        bool _relayState;
        std::string _serviceName;
        ros::ServiceServer _server;

        bool relayCallback(ric_board::RelayRequest &req, ric_board::RelayResponse &res);

    public:
        Relay(byte id, TransportLayer *transportLayer, byte pin, std::string serviceName);
        virtual void write();

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void deviceAck(const DeviceAck *ack);

    protected:
        virtual void buildDevice();

    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_RELAY_H
