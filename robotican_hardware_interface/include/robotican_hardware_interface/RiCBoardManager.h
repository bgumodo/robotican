//
// Created by tom on 08/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
#define ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H

#include <robotican_hardware_interface/TransportLayer.h>
#include <ros/ros.h>

#define MAX_BUFF_SIZE 255
#define PC_VERSION 100

class RiCBoardManager {
private:
    byte _rcvBuff[MAX_BUFF_SIZE];
    TransportLayer _transportLayer;
    unsigned int getBaudrate();
    std::string getPort();

public:
    RiCBoardManager();

    void connect();
    void disconnect();

};

#endif //ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
