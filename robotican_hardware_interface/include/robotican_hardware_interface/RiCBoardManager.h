//
// Created by tom on 08/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
#define ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H

#include <vector>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <robotican_hardware_interface/Device.h>
#include <robotican_hardware_interface/Battery.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <hardware_interface/joint_command_interface.h>
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
        ros::NodeHandle _nodeHandle;
        ros::Timer _sendKeepAliveTimer;
        ros::Timer _timeoutKeepAliveTimer;
        ros::AsyncSpinner _spinner;
        std::vector<Device*> _devices;

        unsigned int getBaudrate();

        std::string getPort();

        void resetBuff();

        ConnectEnum::ConnectEnum getConnectState();

        void setConnectState(ConnectEnum::ConnectEnum connectState);

        void debugMsgHandler(DebugMsg *debugMsg);

        void clear();

    public:
        RiCBoardManager();

        void buildDevices();

        void buildDevices(hardware_interface::JointStateInterface, hardware_interface::VelocityJointInterface);

        void buildDevices(hardware_interface::JointStateInterface, hardware_interface::PositionJointInterface);

        void connect();

        void disconnect();

        void handleMessage();

        void connectionHandle(ConnectState *connectState);

        void sendKeepAliveEvent(const ros::TimerEvent &timerEvent);

        void timeoutKeepAliveEvent(const ros::TimerEvent &timerEvent);

        void keepAliveHandle(KeepAliveMsg *keepAliveMsg);

        void deviceMessageHandler(DeviceMessage *deviceMsg);
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_RICBOARDMANAGER_H
