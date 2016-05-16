//
// Created by tom on 16/05/16.
//

#include<robotican_hardware_interface/Relay.h>

namespace robotican_hardware {
    void Relay::update(const DeviceMessage *deviceMessage) {

    }

    void Relay::write() {
        RelaySetState state;
        state.length = sizeof(state);
        state.checkSum = 0;
        state.id = getId();

        uint8_t *rawData = (uint8_t*)&state;
        state.checkSum = _transportLayer->calcChecksum(rawData, state.length);
        _transportLayer->write(rawData, state.length);
    }

    void Relay::buildDevice() {
        BuildRelay buildRelay;
        buildRelay.length = sizeof(buildRelay);
        buildRelay.checkSum = 0;
        buildRelay.id = getId();
        buildRelay.pin = _pin;

        uint8_t *rawData = (uint8_t*) &buildRelay;
        buildRelay.checkSum = _transportLayer->calcChecksum(rawData, buildRelay.length);
        _transportLayer->write(rawData, buildRelay.length);
    }

    void Relay::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if (isReady()) {
            _server = _nodeHandle.advertiseService(_serviceName, &Relay::relayCallback, this);
            ros_utils::rosInfo("Switch is ready");
        }
        else {
            ros_utils::rosError("RiCBoard can't build relay object for spme reason, this program will shut down now");
            ros::shutdown();
        }
    }

    Relay::Relay(byte id, TransportLayer *transportLayer, byte pin, std::string serviceName)
            : Device(id, transportLayer) {
        _pin = pin;
        _serviceName = serviceName;
        _relayState = false;
        buildDevice();
    }

    bool Relay::relayCallback(ric_board::RelayRequest &req, ric_board::RelayResponse &res) {
        _relayState = req.req;
        res.ack = true;
        return true;
    }
}