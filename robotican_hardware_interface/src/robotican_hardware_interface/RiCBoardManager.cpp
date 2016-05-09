//
// Created by tom on 08/05/16.
//

#include <robotican_hardware_interface/RiCBoardManager.h>

namespace robotican_hardware {
    RiCBoardManager::RiCBoardManager() : _nodeHandle(), _spinner(1) ,_transportLayer(getPort(), getBaudrate()) {
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
}


