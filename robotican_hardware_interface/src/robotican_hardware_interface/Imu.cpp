//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/Imu.h>

namespace robotican_hardware {

    void Imu::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {
            if(deviceMessage->deviceMessageType == DeviceMessageType::ImuFeedback) {
                ImuFeedback *feedback = (ImuFeedback *) deviceMessage;

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
            else if(deviceMessage->deviceMessageType == DeviceMessageType::ImuClibFeedback) {
                if(!_isStopClib) {
                    ImuClibFeedback *clibFeedback = (ImuClibFeedback *) deviceMessage;
                    robotican_hardware_interface::imuClib clibMsg;
                    clibMsg.max.header.frame_id = "base_link";
                    clibMsg.min.header.frame_id = "base_link";
                    clibMsg.max.header.stamp = ros::Time::now();
                    clibMsg.min.header.stamp = ros::Time::now();

                    clibMsg.max.vector.x = clibFeedback->max[0];
                    clibMsg.max.vector.y = clibFeedback->max[1];
                    clibMsg.max.vector.z = clibFeedback->max[2];

                    clibMsg.min.vector.x = clibFeedback->min[0];
                    clibMsg.min.vector.y = clibFeedback->min[1];
                    clibMsg.min.vector.z = clibFeedback->min[2];

                    _clibPub.publish(clibMsg);
                }

            }
        }

    }

    void Imu::write() {
        if(isReady()) {
            if(_isStateChange) {
                _isStateChange = false;
                ImuSetClibState state;
                state.length = sizeof(state);
                state.checkSum = 0;
                state.id = getId();

                state.state = _imuState;
                uint8_t *rawData = (uint8_t*) &state;
                state.checkSum = _transportLayer->calcChecksum(rawData, state.length);
                _transportLayer->write(rawData, state.length);
            }
        }

    }

    void Imu::buildDevice() {
        BuildImu buildImu;
        buildImu.length = sizeof(buildImu);
        buildImu.checkSum = 0;
        buildImu.id = getId();
        buildImu.fusionHz = _fusionHz;
        buildImu.enableGyro =_enableGyro;
        buildImu.fuseCompass =_fuseCompass;

        uint8_t  *rawData = (uint8_t*) &buildImu;

        buildImu.checkSum = _transportLayer->calcChecksum(rawData, buildImu.length);
        _transportLayer->write(rawData, buildImu.length);
        ros::Duration(1.0).sleep();

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

    Imu::Imu(byte id, TransportLayer *transportLayer, uint16_t fusionHz, std::string frameId, bool enableGyro,
                 bool fuseCompass)
            : Device(id, transportLayer) {
        _fusionHz = fusionHz;
        _frameId = frameId;
        _isStopClib = true;
        _isStateChange = false;
        _enableGyro = enableGyro;
        _fuseCompass = fuseCompass;
        _imuState = robotican_hardware_interface::setImuClibRequest::STOP;
        _setImuClibService = _nodeHandle.advertiseService("set_imu_calibration_state", &Imu::onSetImuClib, this);

    }


    bool Imu::onSetImuClib(robotican_hardware_interface::setImuClib::Request &request, robotican_hardware_interface::setImuClib::Response &response) {
        if(request.state == robotican_hardware_interface::setImuClibRequest::CALIBRATION) {
            _isStateChange = _imuState != request.state;
            _isStopClib = false;
            _imuState = robotican_hardware_interface::setImuClibRequest::CALIBRATION;
            _clibPub = _nodeHandle.advertise<robotican_hardware_interface::imuClib>("imu_calibration", 10);
        }
        else if(request.state == robotican_hardware_interface::setImuClibRequest::STOP) {
            _isStateChange = _imuState != request.state;
            _isStopClib = true;
            _imuState = robotican_hardware_interface::setImuClibRequest::STOP;
            _clibPub.shutdown();
        }
        else if(request.state == robotican_hardware_interface::setImuClibRequest::SAVE) {
            _isStateChange = _imuState != request.state;
            _imuState = robotican_hardware_interface::setImuClibRequest::SAVE;
        }
        response.ack = true;
        return true;
    }
}
