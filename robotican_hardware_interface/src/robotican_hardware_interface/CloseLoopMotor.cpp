//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/CloseLoopMotor.h>
namespace robotican_hardware {
//    void CloseLoopMotor::update(const DeviceMessage *deviceMessage) {
//        MotorFeedback* feedback = (MotorFeedback*) deviceMessage;
//        _jointInfo.position = feedback->rad;
//        _jointInfo.velocity = feedback->rad_s;
//    }
//
//    void CloseLoopMotor::write() {
//        MotorSetPoint point;
//        point.length = sizeof(point);
//        point.checkSum = 0;
//        point.id = getId();
//        point.point = (float) _jointInfo.cmd;
//
//        uint8_t *rawData = (uint8_t*) &point;
//
//        point.checkSum = _transportLayer->calcChecksum(rawData, point.length);
//        _transportLayer->write(rawData, point.length);
//    }
//
//    void CloseLoopMotor::buildDevice() {
//        BuildMotorCloseLoop buildMotorCloseLoop;
//        buildMotorCloseLoop.length = sizeof(buildMotorCloseLoop);
//        buildMotorCloseLoop.checkSum = 0;
//        buildMotorCloseLoop.id = getId();
//        buildMotorCloseLoop.encoderPinA = _params.encoderPinA;
//        buildMotorCloseLoop.encoderPinB = _params.encoderPinB;
//        buildMotorCloseLoop.LPFHz = _params.LPFHz;
//        buildMotorCloseLoop.PIDHz = _params.PIDHz;
//        buildMotorCloseLoop.CPR = _params.CPR;
//        buildMotorCloseLoop.timeout = _params.timeout;
//        buildMotorCloseLoop.motorDirection = _params.motorDirection;
//        buildMotorCloseLoop.encoderDirection = _params.encoderDirection;
//        buildMotorCloseLoop.LPFAlpha = _params.LPFAlpha;
//        buildMotorCloseLoop.KP = _params.KP;
//        buildMotorCloseLoop.KI = _params.KI;
//        buildMotorCloseLoop.KD = _params.KD;
//        buildMotorCloseLoop.maxSpeed = _params.maxSpeed;
//        buildMotorCloseLoop.limit = _params.limit;
//
//        uint8_t *rawData = (uint8_t*) &buildMotorCloseLoop;
//        buildMotorCloseLoop.checkSum = _transportLayer->calcChecksum(rawData, buildMotorCloseLoop.length);
//        _transportLayer->write(rawData, buildMotorCloseLoop.length);
//
//    }
//
//    CloseLoopMotor::CloseLoopMotor(byte id, TransportLayer *transportLayer,
//                                   byte motorAddress, CloseMotorParams closeMotorParam) : Motor(id, transportLayer, motorAddress) {
//        _params = closeMotorParam;
//        buildDevice();
//    }
//
//    JointInfo_t *robotican_hardware::CloseLoopMotor::getJointInfo() {
//        return &_jointInfo;
//    }
}