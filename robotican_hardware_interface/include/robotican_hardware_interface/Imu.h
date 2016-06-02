//
// Created by tom on 15/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_IMU_H
#define ROBOTICAN_HARDWARE_INTERFACE_IMU_H


#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/imuClib.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/setImuClib.h>
#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    class Imu : public Device {
    private:
        byte _imuState;
        uint16_t _fusionHz;
        std::string _frameId;
        ros::ServiceServer _setImuClibService;
        ros::Publisher _clibPub;
        ros::Publisher _imuAMQ;
        ros::Publisher _imuM;
        bool _enableGyro;
        bool _fuseCompass;
        bool _isStopClib;

        bool _isStateChange;
        bool onSetImuClib(robotican_hardware_interface::setImuClib::Request &request, robotican_hardware_interface::setImuClib::Response &response);

    public:

        Imu(byte id, TransportLayer *transportLayer, uint16_t fusionHz, std::string frameId, bool enableGyro,
                    bool fuseCompass);

        virtual void update(const DeviceMessage *deviceMessage);

        virtual void write();

        virtual void buildDevice();

        virtual void deviceAck(const DeviceAck *ack);
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_IMU_H
