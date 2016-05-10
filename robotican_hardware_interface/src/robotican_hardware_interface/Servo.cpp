//
// Created by tom on 10/05/16.
//

#include <robotican_hardware_interface/Servo.h>

namespace robotican_hardware {

    Servo::Servo(byte id, TransportLayer *transportLayer, byte pin, float a, float b, float max, float min, float *initPos) : Device(id , transportLayer) {
        _pin = pin;
        _a = a;
        _b = b;
        _max = max;
        _min = min;
        _pos = initPos;
        _command = initPos;
        buildDevice();

    }


    void Servo::update(const DeviceMessage *deviceMessage) {
        ServoFeedback* feedback = (ServoFeedback*) deviceMessage;
        *_pos = feedback->pos;
    }

    void Servo::write() {
        ServoSetPoint point;
        point.length = sizeof(point);
        point.checkSum = 0;
        point.id = getId();
        point.pos = *_command;

        uint8_t *rawData = (uint8_t*)&point;
        point.checkSum = _transportLayer->calcChecksum(rawData, point.length);
        _transportLayer->write(rawData, point.length);
    }

    void Servo::buildDevice() {
        BuildServo buildServo;
        buildServo.length = sizeof(buildServo);
        buildServo.checkSum = 0;
        buildServo.id = getId();
        buildServo.a = _a;
        buildServo.b = _a;
        buildServo.max = _max;
        buildServo.min = _min;

        uint8_t* rawData = (uint8_t*)&buildServo;
        buildServo.checkSum = _transportLayer->calcChecksum(rawData, buildServo.length);
        _transportLayer->write(rawData, buildServo.length);
    }


    void Servo::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Servo is ready");
        }
        else {
            ros_utils::rosError("Can't build Servo for some reason");
            ros::shutdown();
        }
    }
}