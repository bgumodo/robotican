//
// Created by tom on 15/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_GPS_H
#define ROBOTICAN_HARDWARE_INTERFACE_GPS_H

#include <sensor_msgs/NavSatFix.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>


namespace robotican_hardware {
    class Gps : public Device {
    private:
        unsigned int _baudrate;
        std::string _frameId;
        std::string _topicName;
        ros::Publisher _gpsFeedback;
    public:
        Gps(byte id, TransportLayer *transportLayer, unsigned int baudrate, std::string topicName,
            std::string frameId);
        virtual void deviceAck(const DeviceAck *ack);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

    protected:
        virtual void buildDevice();


    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_GPS_H
