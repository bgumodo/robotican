//
// Created by tom on 08/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
#define ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <robotican_hardware_interface/ros_utils.h>
#include <robotican_hardware_interface/TransportLayer.h>

#define MAX_BUFF_SIZE 255
#define PC_VERSION 100
#define RIC_BOARD_DEBUG

namespace robotican_hardware {
    class RiCBoardManager {
    private:
        byte _rcvBuff[MAX_BUFF_SIZE];
        TransportLayer _transportLayer;
        ConnectEnum::ConnectEnum  _connectState;
        unsigned int getBaudrate();
        std::string getPort();
        void resetBuff();
        ConnectEnum::ConnectEnum getConnectState();
        void setConnectState(ConnectEnum::ConnectEnum connectState);


    public:
        RiCBoardManager();

        void connect();

        void disconnect();

        void handleMessage();
        
        void connectionHandle(ConnectState *connectState);
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
