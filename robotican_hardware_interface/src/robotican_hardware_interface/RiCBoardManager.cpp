//
// Created by tom on 08/05/16.
//

#include <robotican_hardware_interface/RiCBoardManager.h>

RiCBoardManager::RiCBoardManager() : _transportLayer(getPort(), getBaudrate()) { }

void RiCBoardManager::connect() {
    ConnectState connectState;
    connectState.length = sizeof(connectState);
    connectState.state = ConnectEnum::Connected;
    connectState.version = PC_VERSION;
    uint8_t *bytes = (uint8_t*)&connectState;
    connectState.checkSum = 0;
    connectState.checkSum = _transportLayer.calcChecksum(bytes, connectState.length);
    _transportLayer.write(bytes, connectState.length);

}

void RiCBoardManager::disconnect() {

    ConnectState connectState;
    connectState.length = sizeof(connectState);
    connectState.state = ConnectEnum::Disconnected;
    connectState.version = PC_VERSION;
    uint8_t *bytes = (uint8_t*)&connectState;
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
