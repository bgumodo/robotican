//
// Created by tom on 09/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_BATTERY_H
#define ROBOTICAN_HARDWARE_INTERFACE_BATTERY_H

#include <ric_board/Battery.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    class Battery : public Device {
    private:
        float _voltageDividerRatio;
        float _max;
        float _min;
        float _currentRead;
        byte _batteryPin;
        ros::Publisher _pub;

    protected:

        virtual void buildDevice();
    public:

        Battery(byte id, float voltageDividerRatio, float max, float min, byte batteryPin,
                        TransportLayer *transportLayer);

        virtual void deviceAck(const DeviceAck *ack);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_BATTERY_H
