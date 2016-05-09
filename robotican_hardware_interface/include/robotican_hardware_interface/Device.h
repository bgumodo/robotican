//
// Created by tom on 09/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H
#define ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H

#include <robotican_hardware_interface/Protocol.h>

namespace robotican_hardware {

    class Device {
    private:
        byte _id;
    public:
        Device(byte id);
        byte getId();
        void setId(byte id);
        virtual void update(const DeviceMessage* deviceMessage)=0;
        virtual void write()=0;
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_IDEVICE_H
