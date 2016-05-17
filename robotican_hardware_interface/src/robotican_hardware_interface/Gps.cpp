//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/Gps.h>
namespace robotican_hardware {
    Gps::Gps(byte id, TransportLayer *transportLayer, unsigned int baudrate, std::string topicName,
             std::string frameId) : Device(id, transportLayer) {
        _baudrate = baudrate;
        _frameId = frameId;
        _topicName = topicName;
    }

    void Gps::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {
            GpsFeedback *feedback = (GpsFeedback *) deviceMessage;
            sensor_msgs::NavSatFix msg;
            msg.header.frame_id = _frameId;
            msg.header.stamp = ros::Time::now();
            msg.latitude = feedback->lat;
            msg.longitude = feedback->lng;
            msg.altitude = feedback->meters;
            msg.status.status = feedback->fix;
            msg.position_covariance[0] = feedback->HDOP * feedback->HDOP;
            msg.position_covariance[4] = feedback->HDOP * feedback->HDOP;
            msg.position_covariance[8] = feedback->HDOP * feedback->HDOP * 4;
            msg.status.service = 1;
            _gpsFeedback.publish(msg);
        }
    }

    void Gps::write() {

    }

    void Gps::buildDevice() {
        BuildGps buildGps;
        buildGps.length = sizeof(buildGps);
        buildGps.checkSum = 0;
        buildGps.id = getId();
        buildGps.baudrate = _baudrate;

        uint8_t  *rawData = (uint8_t*)&buildGps;
        buildGps.checkSum = _transportLayer->calcChecksum(rawData, buildGps.length);
        _transportLayer->write(rawData, buildGps.length);
    }

    void Gps::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Gps is ready");
        }
        else {
            ros_utils::rosError("RiCBoard can't build gps object for spme reason, this program will shut down now");
            ros::shutdown();
        }
    }
}