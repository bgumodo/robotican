//
// Created by tom on 15/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ULTRASONIC_H
#define ROBOTICAN_HARDWARE_INTERFACE_ULTRASONIC_H

#include <sensor_msgs/Range.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/RiCBoardManager.h>

#define URF_HRLV_MaxSonar_an2m 5120.0/65535.0/1000.0
#define MIN_RANGE_URF_HRLV_MaxSonar 0.3f // min 0.16f
#define MAX_RANGE_URF_HRLV_MaxSonar 5.0f //max 6.45f
#define FIELD_OF_VIEW_URF_HRLV_MaxSonar 0.7f //0.7f

#define URF_LV_MaxSonar_an2m 13004.8/65535.0/1000.0
#define MIN_RANGE_URF_LV_MaxSonar 0.16f
#define MAX_RANGE_URF_LV_MaxSonar 6.45f
#define FIELD_OF_VIEW_URF_LV_MaxSonar 0.7f


namespace robotican_hardware {
    class Ultrasonic : public Device {
    private:
        byte _pin;
        std::string _topicName;
        std::string _frameId;
        ros::Publisher _ultrasonicRead;

    protected:
        virtual void buildDevice();

    public:
        Ultrasonic(byte id, TransportLayer *transportLayer, byte pin, std::string topicName, std::string frameId);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        virtual void deviceAck(const DeviceAck *ack);
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_ULTRASONIC_H
