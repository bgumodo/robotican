//
// Created by tom on 08/05/16.
//

#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    RiCBoardManager::RiCBoardManager() : _nodeHandle(), _spinner(1) ,_transportLayer(getPort(), getBaudrate()) {
        _idGen = 0;
        resetBuff();
        setConnectState(ConnectEnum::Disconnected);
        boost::thread thread(&RiCBoardManager::handleMessage, this);
        _timeoutKeepAliveTimer = _nodeHandle.createTimer(ros::Duration(1.0), &RiCBoardManager::timeoutKeepAliveEvent, this);
        _sendKeepAliveTimer = _nodeHandle.createTimer(ros::Duration(1.0), &RiCBoardManager::sendKeepAliveEvent, this);
        _timeoutKeepAliveTimer.stop();
        _sendKeepAliveTimer.stop();
        _spinner.start();

    }

    void RiCBoardManager::connect() {
        ConnectState connectState;
        connectState.length = sizeof(connectState);
        connectState.state = ConnectEnum::Connected;
        connectState.version = PC_VERSION;
        uint8_t *bytes = (uint8_t *) &connectState;
        connectState.checkSum = 0;
        connectState.checkSum = _transportLayer.calcChecksum(bytes, connectState.length);
        _transportLayer.write(bytes, connectState.length);

    }

    void RiCBoardManager::disconnect() {

        ConnectState connectState;
        connectState.length = sizeof(connectState);
        connectState.state = ConnectEnum::Disconnected;
        connectState.version = PC_VERSION;
        uint8_t *bytes = (uint8_t *) &connectState;
        connectState.checkSum = 0;
        connectState.checkSum = _transportLayer.calcChecksum(bytes, connectState.length);
        _transportLayer.write(bytes, connectState.length);
        clear();

    }

    unsigned int RiCBoardManager::getBaudrate() {
        int baudrate;
        ros::param::param<int>("baudrate", baudrate, 9600);
        return (unsigned int) baudrate;
    }

    std::string RiCBoardManager::getPort() {
        std::string post;
        ros::param::param<std::string>("port", post, "/dev/RiCBoard");
        return post;

    }

    void RiCBoardManager::handleMessage() {
        while(ros::ok()) {
            if(_transportLayer.tryToRead(_rcvBuff, MAX_BUFF_SIZE)) {
                Header *header = (Header*) _rcvBuff;
                crc prevCheckSum = header->checkSum;
                header->checkSum = 0;
                crc curCheckSum = _transportLayer.calcChecksum(_rcvBuff, header->length);
                if(curCheckSum == prevCheckSum) {
                    switch (header->dataType) {
                        case DataType::ConnectionState:
                            connectionHandle((ConnectState*)header);
                            break;
                        case DataType::Debug:
                            debugMsgHandler((DebugMsg*) header);
                            break;
                        case DataType::KeepAlive:
                            keepAliveHandle((KeepAliveMsg*)header);
                            break;
                        case DataType::DeviceMessage:
                            deviceMessageHandler((DeviceMessage *)header);
                            break;
                        default:
                            break;
                    }

                }
                else {
#ifdef RIC_BOARD_DEBUG
                    char errorBuff[128] = {'\0'};
                    sprintf(errorBuff, "Invalid checksum {cur: %d, prev: %d}", curCheckSum, prevCheckSum);
                    ros_utils::rosError(errorBuff);
#endif
                }
                resetBuff();
            }

        }

        _timeoutKeepAliveTimer.stop();
        _sendKeepAliveTimer.stop();
    }

    void RiCBoardManager::resetBuff() {
        for(int i = 0; i < MAX_BUFF_SIZE; ++i) {
            _rcvBuff[i] = 0;
        }
    }

    ConnectEnum::ConnectEnum RiCBoardManager::getConnectState() {
        return _connectState;
    }

    void RiCBoardManager::setConnectState(ConnectEnum::ConnectEnum connectState) {
        _connectState = connectState;
    }

    void RiCBoardManager::connectionHandle(ConnectState *connectState) {
        switch(connectState->state) {
            case ConnectEnum::Connected:
                if(getConnectState() == ConnectEnum::Disconnected) {
                    setConnectState(ConnectEnum::Connected);
                    _sendKeepAliveTimer.start();
                    _timeoutKeepAliveTimer.setPeriod(ros::Duration(3.0), true);
                    _timeoutKeepAliveTimer.start();
                    ros_utils::rosInfo("Handshake complete: RiCBoard is connected");

                }
                break;
            case ConnectEnum::NotReady:
                break;
            case ConnectEnum::AlreadyConnected:
                break;
            case ConnectEnum::Disconnected:
                if(getConnectState() == ConnectEnum::Connected) {
                    setConnectState(ConnectEnum::Disconnected);
                    ros_utils::rosInfo("RiCBoard is now disconnected");
                    ros::shutdown();
                }
                break;
            case ConnectEnum::AlreadyDisconnected:
                break;
            default:break;
        }
    }

    void RiCBoardManager::debugMsgHandler(DebugMsg *debugMsg) {
        switch (debugMsg->level) {
            case DebugLevel::Info:
                ros_utils::rosInfo(debugMsg->message);
                break;
            case DebugLevel::Warn:
                ros_utils::rosWarn(debugMsg->message);
                break;
            case DebugLevel::Error:
                ros_utils::rosError(debugMsg->message);
                break;
            case DebugLevel::Fatal:
                ros_utils::rosError(debugMsg->message);
                break;
            default:break;
        }
    }

    void RiCBoardManager::sendKeepAliveEvent(const ros::TimerEvent &timerEvent) {
        KeepAliveMsg keepAliveMsg;
        keepAliveMsg.length = sizeof(keepAliveMsg);
        keepAliveMsg.checkSum = 0;
        keepAliveMsg.state = KeepAliveState::Ok;
        uint8_t* rawData = (uint8_t*) &keepAliveMsg;
        keepAliveMsg.checkSum = _transportLayer.calcChecksum(rawData, keepAliveMsg.length);
        _transportLayer.write(rawData, keepAliveMsg.length);
    }

    void RiCBoardManager::timeoutKeepAliveEvent(const ros::TimerEvent &timerEvent) {
        ros_utils::rosError("RiCBoard not responding. Shuting down the program");
        ros::shutdown();
        clear();
    }

    void RiCBoardManager::keepAliveHandle(KeepAliveMsg *keepAliveMsg) {
        switch(keepAliveMsg->state) {
            case KeepAliveState::Ok:
                _timeoutKeepAliveTimer.setPeriod(ros::Duration(3.0), true);
                break;
            case KeepAliveState::NeedToRestart:
                break;
            case KeepAliveState::FatalError:
                break;
            default:
                break;
        }

    }

    void RiCBoardManager::buildDevices() {

#ifndef RIC_BOARD_DEBUG
        if(_nodeHandle.hasParam("battery_pin")) {
            int pin;
            float voltageDividerRatio, max, min;
            if(_nodeHandle.getParam("battery_pin", pin)
               && _nodeHandle.getParam("battery_voltage_divider_ratio", voltageDividerRatio)
               && _nodeHandle.getParam("battery_max", max)
               && _nodeHandle.getParam("battery_min", min)) {
                Device *battery = new Battery(_idGen++, voltageDividerRatio, max, min, (byte) pin, &_transportLayer);
                _devices.push_back(battery);
            }
            else {
                ros_utils::rosError("Can't build battery some of the parameters are missing");
            }
        }

        int ultrasonicSize = 0;
        ros::param::param("ultrasonic_size", ultrasonicSize, 0);
        for(int i = 0; i < ultrasonicSize; ++i) {
            std::string ultrasonicIdentifier = "ultrasoic" + i, frameId, topicName;
            int pin;
            if(_nodeHandle.getParam(ultrasonicIdentifier + "_pin", pin)
               && _nodeHandle.getParam(ultrasonicIdentifier + "_frame_id", frameId)
               && _nodeHandle.getParam(ultrasonicIdentifier + "_topic_name", topicName)) {
                Device *ultrasonic = new Ultrasonic(_idGen++, &_transportLayer, pin,  topicName, frameId);
                _devices.push_back(ultrasonic);
            }
        }

        if(_nodeHandle.hasParam("imu_fusion_hz")) {
            int fusionHz;
            bool enableGyro;
            std::string frameId;

            if(_nodeHandle.getParam("imu_fusion_hz", fusionHz)
               && _nodeHandle.getParam("imu_enable_gyro", enableGyro)
               && _nodeHandle.getParam("imu_frame_id", frameId)) {
                Device *imu = new Imu(_idGen++, &_transportLayer, (uint16_t) fusionHz, frameId, enableGyro);
                _devices.push_back(imu);
            }
        }

        if(_nodeHandle.hasParam("gps_baudrate")) {
            int baudrate;
            std::string frameId, topicName;

            if(_nodeHandle.getParam("gps_baudrate", baudrate)
               && _nodeHandle.getParam("gps_topic_name", topicName)
               && _nodeHandle.getParam("gps_frame_id", frameId)) {
                Device *gps = new Gps(_idGen++, &_transportLayer, (uint16_t) baudrate, topicName, frameId);
                _devices.push_back(gps);
            }
        }


#endif
#ifdef  RIC_BOARD_DEBUG
                int pin;
                float voltageDividerRatio, max, min;
                _nodeHandle.param<int>("battery_pin", pin, 17);
                _nodeHandle.param<float>("battery_voltage_divider_ratio", voltageDividerRatio, 6.0);
                _nodeHandle.param<float>("battery_max", max, 11.3);
                _nodeHandle.param<float>("battery_min", min, 10.0);
                Device *battery = new Battery(_idGen++, voltageDividerRatio, max, min, (byte) pin, &_transportLayer);
                _devices.push_back(battery);

#endif

    }

    void RiCBoardManager::deviceMessageHandler(DeviceMessage *deviceMsg) {
        size_t devicesSize = _devices.size();
        if(devicesSize > deviceMsg->id) {
            switch ((DeviceMessageType::DeviceMessageType) deviceMsg->deviceMessageType) {
                case DeviceMessageType::BuildDevice:
                    break;
                case DeviceMessageType::Ack:
                    _devices[deviceMsg->id]->deviceAck((DeviceAck*) deviceMsg);
                    break;
                case DeviceMessageType::MotorSetPointMsg:
                    break;
                case DeviceMessageType::MotorFeedback:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
                case DeviceMessageType::MotorSetPid:
                    break;
                case DeviceMessageType::ServoFeedback:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
                case DeviceMessageType::ServoSetPoint:
                    break;
                case DeviceMessageType::SwitchFeedBack:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
                case DeviceMessageType::UltrasonicFeedback:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
                case DeviceMessageType::RelySetState:
                    break;
                case DeviceMessageType::GpsFeedback:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
                case DeviceMessageType::ImuFeedback:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
                case DeviceMessageType::BatteryFeedback:
                    _devices[deviceMsg->id]->update(deviceMsg);
                    break;
            }
        }

    }

    void RiCBoardManager::clear() {
        size_t size = _devices.size();
        if(size > 0) {
            for(int i = 0; i < size; ++i) _devices[i];
            _devices.clear();
        }

    }


    void RiCBoardManager::buildDevices(hardware_interface::JointStateInterface* jointStateInterface,
                                       hardware_interface::PositionJointInterface* jointPositionInterface) {
        int servoSize = 0;
        ros::param::param<int>("servo_size", servoSize, 0);
        for(int i = 0; i < servoSize; ++i) {
            std::string servoIdentifier = "servo" + i, servoJointName = "";
            float a , b,  max,  min,  initPos;
            int pin;

            if(_nodeHandle.getParam(servoIdentifier + "_a", a)
               &&_nodeHandle.getParam(servoIdentifier + "_b", b)
               &&_nodeHandle.getParam(servoIdentifier + "_max", max)
               &&_nodeHandle.getParam(servoIdentifier + "_min", min)
               &&_nodeHandle.getParam(servoIdentifier + "_init_pos", initPos)
               &&_nodeHandle.getParam(servoIdentifier + "_pin", pin)
               &&_nodeHandle.getParam(servoIdentifier + "_joint_name", servoJointName)) {
                Servo* servo = new Servo(_idGen++, &_transportLayer, (byte) pin, a, b, max, min, initPos);
                JointInfo_t* servoJointInfo = servo->getJointInfo();
                hardware_interface::JointStateHandle jointStateHandle(servoJointName,
                                                                      &servoJointInfo->position,
                                                                      &servoJointInfo->velocity,
                                                                      &servoJointInfo->effort);


                jointStateInterface->registerHandle(jointStateHandle);
                hardware_interface::JointHandle JointHandle(jointStateInterface->getHandle(servoJointName),
                                                                &servoJointInfo->cmd);
                jointPositionInterface->registerHandle(JointHandle);
                _devices.push_back(servo);
            }
        }


    }

    void RiCBoardManager::buildDevices(hardware_interface::JointStateInterface *jointStateInterface,
                                       hardware_interface::VelocityJointInterface *jointVelocityInterface) {
        int closeMotorSize = 0;
        ros::param::param<int>("close_motor_size", closeMotorSize, 0);
        for(int i = 0; i < closeMotorSize; ++i) {
            std::string closeMotorIdentifier = "motor" + i, jointName;
            CloseMotorParams motorParams;

            int encoderPinA, encoderPinB, LPFHz, PIDHz, CPR, timeout, motorDirection, encoderDirection, motorAddress, eSwitchPin, eSwitchType;
            float LPFAlpha, KP, KI, KD, maxSpeed, limit;

            if(_nodeHandle.getParam(closeMotorIdentifier + "_encoder_pin_A", encoderPinA)
               && _nodeHandle.getParam(closeMotorIdentifier + "_encoder_pin_B", encoderPinB)
               && _nodeHandle.getParam(closeMotorIdentifier + "_lpf_hz", LPFHz)
               && _nodeHandle.getParam(closeMotorIdentifier + "_pid_hz", PIDHz)
               && _nodeHandle.getParam(closeMotorIdentifier + "_cpr", CPR)
               && _nodeHandle.getParam(closeMotorIdentifier + "_timeout", timeout)
               && _nodeHandle.getParam(closeMotorIdentifier + "_motor_direction", motorDirection)
               && _nodeHandle.getParam(closeMotorIdentifier + "_encoder_direction", encoderDirection)
               && _nodeHandle.getParam(closeMotorIdentifier + "_lpf_alpha", LPFAlpha)
               && _nodeHandle.getParam(closeMotorIdentifier + "_kp", KP)
               && _nodeHandle.getParam(closeMotorIdentifier + "_ki", KI)
               && _nodeHandle.getParam(closeMotorIdentifier + "_kd", KD)
               && _nodeHandle.getParam(closeMotorIdentifier + "_max_speed", maxSpeed)
               && _nodeHandle.getParam(closeMotorIdentifier + "_limit", limit)
               && _nodeHandle.getParam(closeMotorIdentifier + "_motor_address", motorAddress)
               && _nodeHandle.getParam(closeMotorIdentifier + "_motor_emergency_pin", eSwitchPin)
               && _nodeHandle.getParam(closeMotorIdentifier + "_motor_emergency_pin_type", eSwitchType)
               && _nodeHandle.getParam(closeMotorIdentifier + "_joint", jointName)) {

                motorParams.encoderPinA = (byte) encoderPinA;
                motorParams.encoderPinB = (byte) encoderPinB;
                motorParams.LPFHz = (uint16_t) LPFHz;
                motorParams.PIDHz = (uint16_t) PIDHz;
                motorParams.CPR = (uint16_t) CPR;
                motorParams.timeout = (uint16_t) timeout;
                motorParams.motorDirection = motorDirection;
                motorParams.encoderDirection = encoderDirection;
                motorParams.LPFAlpha = LPFAlpha;
                motorParams.KP = KP;
                motorParams.KI = KI;
                motorParams.KD = KD;
                motorParams.maxSpeed = maxSpeed;
                motorParams.limit = limit;

                CloseLoopMotor* closeLoopMotor = new CloseLoopMotor(_idGen++, &_transportLayer, (byte) motorAddress, eSwitchPin,
                                                                    eSwitchType, motorParams);
                JointInfo_t* jointInfo = closeLoopMotor->getJointInfo();

                hardware_interface::JointStateHandle jointStateHandle(jointName,
                                                                      &jointInfo->position,
                                                                      &jointInfo->velocity,
                                                                      &jointInfo->effort);

                jointStateInterface->registerHandle(jointStateHandle);

                hardware_interface::JointHandle JointHandle(jointStateInterface->getHandle(jointName),
                                                            &jointInfo->cmd);
                jointVelocityInterface->registerHandle(JointHandle);

                _devices.push_back(closeLoopMotor);
            }
            
        }

        int openLoopSize = 0;
        ros::param::param<int>("open_motor_size", openLoopSize, 0);
        for(int i = 0; i < openLoopSize; ++i) {
            std::string openMotorIdentifier = "motor" + i, jointName;
            int motorAddress, eSwitchPin, eSwitchType;
            if(_nodeHandle.getParam(openMotorIdentifier + "_motor_address", motorAddress)
               && _nodeHandle.getParam(openMotorIdentifier + "_motor_emergency_pin", eSwitchPin)
               && _nodeHandle.getParam(openMotorIdentifier + "_motor_emergency_pin_type", eSwitchType)
               && _nodeHandle.getParam(openMotorIdentifier + "_joint", jointName)) {
                OpenLoopMotor* openLoopMotor = new OpenLoopMotor(_idGen++, &_transportLayer, (byte) motorAddress,
                                                                 (byte) eSwitchPin, (byte) eSwitchType);
                JointInfo_t* jointInfo = openLoopMotor->getJointInfo();

                hardware_interface::JointStateHandle jointStateHandle(jointName,
                                                                      &jointInfo->position,
                                                                      &jointInfo->velocity,
                                                                      &jointInfo->effort);

                jointStateInterface->registerHandle(jointStateHandle);

                hardware_interface::JointHandle JointHandle(jointStateInterface->getHandle(jointName),
                                                            &jointInfo->cmd);
                jointVelocityInterface->registerHandle(JointHandle);

                _devices.push_back(openLoopMotor);

            }
        }

    }

    void RiCBoardManager::write() {
        if(ros::ok()) {
            size_t deviceSize = _devices.size();
            for(int i = 0; i < deviceSize; ++i)
                _devices[i]->write();
        }

    }
}


