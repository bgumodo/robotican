//
// Created by tom on 09/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H
#define ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H

#include <ros/ros.h>
#include <robotican_hardware_interface/Protocol.h>
#include <robotican_hardware_interface/TransportLayer.h>

namespace robotican_hardware {

    class Device {
    private:
        byte _id;
        bool _ready;
    protected:
        ros::NodeHandle _nodeHandle;
        TransportLayer* _transportLayer;
        virtual void buildDevice() =0;
    public:
        Device(byte id, TransportLayer *transportLayer);
        byte getId();
        void setId(byte id);
        bool isReady();
        void setReady(bool ready);
        virtual void deviceAck(const DeviceAck *ack);
        virtual void update(const DeviceMessage * deviceMessage)=0;
        virtual void write() =0;
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H
