//
// Created by tom on 15/05/16.
//

#include <robotican_hardware_interface/RiCMotor.h>
namespace robotican_hardware {

    void RiCMotor::deviceAck(const DeviceAck *ack) {
        Device::deviceAck(ack);
        if(isReady()) {
            ros_utils::rosInfo("Motor is ready");
        }
        else {
            ros_utils::rosError("RiCBoard can't build motor object for spme reason, this program will shut down now");
            ros::shutdown();
        }
    }

    RiCMotor::RiCMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType)
            : Device(id, transportLayer) {
        _motorAddress = motorAddress;
        _eSwitchPin = eSwitchPin;
        _eSwitchType = eSwitchType;
    }



    CloseLoopMotor::CloseLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin, byte eSwitchType,
                                       CloseMotorType::CloseMotorType motorType, CloseMotorMode::CloseMotorMode mode)
            : RiCMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType) {
        _motorType = motorType;
        _mode = mode;
    }

    JointInfo_t *CloseLoopMotor::getJointInfo() {
        return &_jointInfo;
    }

    void CloseLoopMotor::update(const DeviceMessage *deviceMessage) {
        if(isReady()) {
            MotorFeedback *feedback = (MotorFeedback *) deviceMessage;
            _jointInfo.position = feedback->rad;
            _jointInfo.velocity = feedback->rad_s;
        }
    }

    void CloseLoopMotor::write() {
        MotorSetPoint point;
        point.length = sizeof(point);
        point.checkSum = 0;
        point.point = (float) _jointInfo.cmd;

        uint8_t *rawData = (uint8_t*) &point;
        point.checkSum = _transportLayer->calcChecksum(rawData, point.length);
        _transportLayer->write(rawData, point.length);
    }

    CloseMotorType::CloseMotorType CloseLoopMotor::getCloseMotorType() {
        return _motorType;
    }

    CloseMotorMode::CloseMotorMode CloseLoopMotor::getMode() {
        return _mode;
    }


    byte RiCMotor::getESwitchPin() {
        return _eSwitchPin;
    }

    byte RiCMotor::getESwitchType() {
        return _eSwitchType;
    }

    byte RiCMotor::getAddress() {
        return _motorAddress;
    }

    OpenLoopMotor::OpenLoopMotor(byte id, TransportLayer *transportLayer, byte motorAddress, byte eSwitchPin,
                                     byte eSwitchType, float maxSpeed) : RiCMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType) {

        _maxSpeed = maxSpeed;

        buildDevice();
    }

    void OpenLoopMotor::update(const DeviceMessage *deviceMessage) {

    }

    void OpenLoopMotor::write() {
        MotorSetPoint motorSetPoint;
        motorSetPoint.length = sizeof(motorSetPoint);
        motorSetPoint.checkSum = 0;
        motorSetPoint.id = getId();
        motorSetPoint.point = (float) _jointInfo.cmd;

        uint8_t* rawData = (uint8_t*) &motorSetPoint;
        motorSetPoint.checkSum = _transportLayer->calcChecksum(rawData, motorSetPoint.length);
        _transportLayer->write(rawData, motorSetPoint.length);
    }

    void OpenLoopMotor::buildDevice() {
        BuildMotorOpenLoop buildMotorOpenLoop;
        buildMotorOpenLoop.length = sizeof(buildMotorOpenLoop);
        buildMotorOpenLoop.checkSum = 0;
        buildMotorOpenLoop.id = getId();
        buildMotorOpenLoop.motorAddress = getAddress();
        buildMotorOpenLoop.eSwitchPin = getESwitchPin();
        buildMotorOpenLoop.eSwitchType = getESwitchType();
        buildMotorOpenLoop.maxSpeed = _maxSpeed;

        uint8_t* rawData = (uint8_t*) &buildMotorOpenLoop;
        buildMotorOpenLoop.checkSum = _transportLayer->calcChecksum(rawData, buildMotorOpenLoop.length);
        _transportLayer->write(rawData, buildMotorOpenLoop.length);
    }

    JointInfo_t *OpenLoopMotor::getJointInfo() {
        return &_jointInfo;
    }


    CloseLoopMotorWithEncoder::CloseLoopMotorWithEncoder(byte id, TransportLayer *transportLayer, byte motorAddress,
                                                             byte eSwitchPin, byte eSwitchType,
                                                             CloseMotorType::CloseMotorType motoryType,
                                                             CloseMotorMode::CloseMotorMode mode,
                                                             CloseMotorWithEncoderParam param)
            : CloseLoopMotor(id, transportLayer, motorAddress, eSwitchPin, eSwitchType, motoryType, mode) {
        _params = param;
        buildDevice();

    }

    void CloseLoopMotorWithEncoder::buildDevice() {
        BuildMotorCloseLoopWithEncoder buildMotorCloseLoop;
        buildMotorCloseLoop.length = sizeof(buildMotorCloseLoop);
        buildMotorCloseLoop.checkSum = 0;
        buildMotorCloseLoop.id = getId();
        buildMotorCloseLoop.motorAddress = getAddress();
        buildMotorCloseLoop.eSwitchPin = getESwitchPin();
        buildMotorCloseLoop.eSwitchType = getESwitchType();
        buildMotorCloseLoop.encoderPinA = _params.encoderPinA;
        buildMotorCloseLoop.encoderPinB = _params.encoderPinB;
        buildMotorCloseLoop.LPFHz = _params.LPFHz;
        buildMotorCloseLoop.PIDHz = _params.PIDHz;
        buildMotorCloseLoop.PPR = _params.PPR;
        buildMotorCloseLoop.timeout = _params.timeout;
        buildMotorCloseLoop.motorDirection = _params.motorDirection;
        buildMotorCloseLoop.encoderDirection = _params.encoderDirection;
        buildMotorCloseLoop.LPFAlpha = _params.LPFAlpha;
        buildMotorCloseLoop.KP = _params.KP;
        buildMotorCloseLoop.KI = _params.KI;
        buildMotorCloseLoop.KD = _params.KD;
        buildMotorCloseLoop.maxSpeed = _params.maxSpeed;
        buildMotorCloseLoop.limit = _params.limit;
        buildMotorCloseLoop.motorType = getCloseMotorType();
        buildMotorCloseLoop.motorMode = getMode();

        uint8_t* rawData = (uint8_t*) &buildMotorCloseLoop;
        buildMotorCloseLoop.checkSum = _transportLayer->calcChecksum(rawData, buildMotorCloseLoop.length);
        _transportLayer->write(rawData, buildMotorCloseLoop.length);
    }

}