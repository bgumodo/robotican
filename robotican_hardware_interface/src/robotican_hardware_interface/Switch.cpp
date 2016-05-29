//
// Created by tom on 16/05/16.
//

#include <robotican_hardware_interface/Switch.h>

void robotican_hardware::Switch::update(const DeviceMessage* deviceMessage) {
    if(isReady()) {
        SwitchFeedback *feedback = (SwitchFeedback *) deviceMessage;
        std_msgs::Bool state;
        state.data = feedback->state;
        _switchState.publish(state);
    }
}

void robotican_hardware::Switch::write() {

}

void robotican_hardware::Switch::buildDevice() {
    BuildSwitch buildSwitch;
    buildSwitch.length = sizeof(buildSwitch);
    buildSwitch.checkSum = 0;
    buildSwitch.id = getId();
    buildSwitch.pin = _pin;

    uint8_t *rawData = (uint8_t*) &buildSwitch;
    buildSwitch.checkSum = _transportLayer->calcChecksum(rawData, buildSwitch.length);
    _transportLayer->write(rawData, buildSwitch.length);

}

robotican_hardware::Switch::Switch(byte id, TransportLayer *transportLayer, byte pin, std::string topicName) : Device(id, transportLayer) {
    _pin = pin;
    _topicName = topicName;
}

void robotican_hardware::Switch::deviceAck(const DeviceAck *ack) {
    Device::deviceAck(ack);
    if(isReady()) {
        _switchState = _nodeHandle.advertise<std_msgs::Bool>(_topicName, 10);
        ros_utils::rosInfo("Switch is ready");
    }
    else {
        ros_utils::rosError("RiCBoard can't build Switch object for spme reason, this program will shut down now");
        ros::shutdown();
    }
}


