//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/Imu.h>

namespace robotican_hardware {

    void Imu::update(const DeviceMessage *deviceMessage) {
        ImuFeedback *feedback = (ImuFeedback*) deviceMessage;

        sensor_msgs::Imu imuMsg;
        imuMsg.header.frame_id = _frameId;
        imuMsg.header.stamp = ros::Time::now();
        imuMsg.orientation.x = feedback->orientationX;
        imuMsg.orientation.y = feedback->orientationY;
        imuMsg.orientation.z = feedback->orientationZ;
        imuMsg.orientation.w = feedback->orientationW;
        imuMsg.linear_acceleration.x = feedback->accelerationX;
        imuMsg.linear_acceleration.y = feedback->accelerationY;
        imuMsg.linear_acceleration.z = feedback->accelerationZ;
        imuMsg.angular_velocity.x = feedback->velocityX;
        imuMsg.angular_velocity.y = feedback->velocityY;
        imuMsg.angular_velocity.z = feedback->velocityZ;

        sensor_msgs::MagneticField magneticField;
        magneticField.header.frame_id = _frameId;
        magneticField.header.stamp = ros::Time::now();
        magneticField.magnetic_field.x = feedback->magnetometerX;
        magneticField.magnetic_field.y = feedback->magnetometerY;
        magneticField.magnetic_field.z = feedback->magnetometerZ;

        _imuAMQ.publish(imuMsg);
        _imuM.publish(magneticField);


    }

    void Imu::write() {

    }

    void Imu::buildDevice() {
        BuildImu buildImu;
        buildImu.length = sizeof(buildImu);
        buildImu.checkSum = 0;
        buildImu.id = getId();
        buildImu.fusionHz = _fusionHz;
        buildImu.enableGyro =_enableGyro;

        uint8_t  *rawData = (uint8_t*) &buildImu;

        buildImu.checkSum = _transportLayer->calcChecksum(rawData, buildImu.length);
        _transportLayer->write(rawData, buildImu.length);

    }


    void Imu::deviceAck(const DeviceAck* ack){
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Imu is ready");
            _imuAMQ = _nodeHandle.advertise<sensor_msgs::Imu>("imu", 10);
            _imuM = _nodeHandle.advertise<sensor_msgs::MagneticField>("imu_M", 10);
        }
        else {
            ros_utils::rosError("RiCBoard can't build Imu object for spme reason, this program will shut down now");
            ros::shutdown();
        }
    }

    Imu::Imu(byte id, TransportLayer *transportLayer, uint16_t fusionHz, std::string frameId, bool enableGyro)
            : Device(id, transportLayer) {
        _fusionHz = fusionHz;
        _frameId = frameId;
        _enableGyro = enableGyro;
        buildDevice();
    }


}
