//
// Created by tom on 09/05/16.
//

#include <robotican_hardware_interface/Battery.h>

namespace robotican_hardware {

    void Battery::write() {

    }

    void Battery::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {
            BatteryFeedback *feedback = (BatteryFeedback *) deviceMessage;
            _currentRead = feedback->currentRead;
            ric_board::Battery msg;
            msg.data = (float) (_currentRead * 3.3 / 65535.0 * _voltageDividerRatio);
            msg.max = _max;
            msg.min = _min;
            _pub.publish(msg);
        }
    }

    Battery::Battery(byte id, float voltageDividerRatio, float max, float min, byte batteryPin,
                         TransportLayer *transportLayer) : Device(id,transportLayer) {
        _voltageDividerRatio = voltageDividerRatio;
        _max = max;
        _min = min;
        _currentRead = 0;
        _batteryPin = batteryPin;
    }

    void Battery::buildDevice() {
        BuildBattery buildBattery;
        buildBattery.id = getId();
        buildBattery.length = sizeof(buildBattery);
        buildBattery.pin = _batteryPin;
        buildBattery.checkSum = 0;

        byte *rawData = (byte*)&buildBattery;

        buildBattery.checkSum = _transportLayer->calcChecksum(rawData, buildBattery.length);
        _transportLayer->write(rawData, buildBattery.length);
    }

    void Battery::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Battery is ready");
            _pub = _nodeHandle.advertise<ric_board::Battery>("battery_monitor", 10);
        }
        else {
            ros_utils::rosError("RiCBoard can't build battery object for spme reason, this program will shut down now");
            ros::shutdown();
        }
    }
}
