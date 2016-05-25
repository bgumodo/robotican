//
// Created by tom on 15/05/16.
//
#include <robotican_hardware_interface/Ultrasonic.h>

robotican_hardware::Ultrasonic::Ultrasonic(byte id, TransportLayer *transportLayer, byte pin, std::string topicName, std::string frameId)
        : Device(id, transportLayer) {
    _pin = pin;
    _topicName = topicName;
    _frameId = frameId;

}

void robotican_hardware::Ultrasonic::update(const DeviceMessage *deviceMessage) {
    if(isReady()) {
        UltrasonicFeedback *feedback = (UltrasonicFeedback *) deviceMessage;
        uint16_t currentRead = feedback->currentRead;
        sensor_msgs::Range range;
        range.header.frame_id = _frameId;
        range.header.stamp = ros::Time::now();

        range.max_range = MAX_RANGE_URF_HRLV_MaxSonar;
        range.min_range = MIN_RANGE_URF_HRLV_MaxSonar;
        range.field_of_view = FIELD_OF_VIEW_URF_HRLV_MaxSonar;

        range.radiation_type = sensor_msgs::Range::ULTRASOUND;
        range.range = (float) currentRead * URF_HRLV_MaxSonar_an2m;
        _ultrasonicRead.publish(range);
    }

}

void robotican_hardware::Ultrasonic::write() {

}

void robotican_hardware::Ultrasonic::buildDevice() {
    BuildUltrasonic buildUltrasonic;
    buildUltrasonic.length = sizeof(buildUltrasonic);
    buildUltrasonic.checkSum = 0;
    buildUltrasonic.id = getId();
    buildUltrasonic.pin = _pin;

    uint8_t *rawData = (uint8_t*)&buildUltrasonic;
    buildUltrasonic.checkSum = _transportLayer->calcChecksum(rawData, buildUltrasonic.length);
    _transportLayer->write(rawData, buildUltrasonic.length);
}

void robotican_hardware::Ultrasonic::deviceAck(const DeviceAck *ack) {
    Device::deviceAck(ack);
    if(isReady()) {
        ros_utils::rosInfo("Ultrasonic is ready");
        _ultrasonicRead = _nodeHandle.advertise<sensor_msgs::Range>(_topicName, 10);

    }
    else {
        ros_utils::rosError("RiCBoard can't build ultrasonic object for spme reason, this program will shut down now");
        ros::shutdown();
    }
}
