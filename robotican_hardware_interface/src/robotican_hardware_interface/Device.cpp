//
// Created by tom on 09/05/16.
//

#include <robotican_hardware_interface/Device.h>
namespace robotican_hardware {
    Device::Device(byte id, TransportLayer *transportLayer) {
        _id = id;
        _ready = false;
        _transportLayer = transportLayer;
    }

    byte Device::getId() {
        return _id;
    }

    void Device::setId(byte id) {
        _id = id;
    }

    void Device::setReady(bool ready) {
        _ready = ready;
    }

    bool Device::isReady() {
        return _ready;
    }

    void Device::deviceAck(const DeviceAck *ack) {
        _ready = ack->ackId == _id;

    }
}
