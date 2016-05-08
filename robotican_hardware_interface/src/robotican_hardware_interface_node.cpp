//
// Created by tom on 06/04/16.
//



#include <ros/ros.h>
#include <robotican_hardware_interface/robot_base.h>
#include <robotican_hardware_interface/armadillo.h>
#include <robotican_hardware_interface/LiziRobot.h>
#include <robotican_hardware_interface/komodo.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <controller_manager/controller_manager.h>
#include <robotican_hardware_interface/TransportLayer.h>

#define PC_VERSION 100
//#define RIC_BOARD_TEST

int main(int argc, char **argv) {

    ros::init(argc, argv, "robotican_hardware_interface_node");
    ros::NodeHandle nodeHandle;
#ifdef RIC_BOARD_TEST
    TransportLayer transportLayer("/dev/RiCBoard", 9600);
    ConnectState connectState;
    connectState.length = sizeof(connectState);
    connectState.state = ConnectEnum::Connected;
    connectState.version = PC_VERSION;
    uint8_t *bytes = (uint8_t*)&connectState;
    connectState.checkSum = 0;
    connectState.checkSum = transportLayer.calcChecksum(bytes, connectState.length);
    transportLayer.write(bytes, connectState.length);

    byte rcv[128];
    if(transportLayer.tryToRead(rcv, 128)) {
        Header* header = (Header*) rcv;

        crc prvCheckSum = header->checkSum, curCheckSum;
        header->checkSum = 0;
        curCheckSum = transportLayer.calcChecksum(rcv, header->length);
        if(curCheckSum == prvCheckSum) {
            switch (header->dataType) {
                case DataType::ConnectionState: {
                    ConnectState *replay = (ConnectState *) header;
                    if (replay->state == ConnectEnum::Connected) {
                        ROS_INFO("[%s]: Handshake complete: RiCBoard now connected", ros::this_node::getName().c_str());
                        ros::Duration(2.0).sleep();

                        ConnectState connectState;
                        connectState.length = sizeof(connectState);
                        connectState.state = ConnectEnum::Disconnected;
                        connectState.version = PC_VERSION;
                        uint8_t *bytes = (uint8_t*)&connectState;
                        connectState.checkSum = 0;
                        connectState.checkSum = transportLayer.calcChecksum(bytes, connectState.length);
                        transportLayer.write(bytes, connectState.length);
                    }
                    else {
                        ROS_ERROR("[%s]: Handshake error: RiCBoard is now in state %d",
                                  ros::this_node::getName().c_str(), replay->state);
                    }
                }
                    break;
                default:
                        ROS_ERROR("[%s]: Expected Connection state insted got %d", ros::this_node::getName().c_str(), header->dataType);
                    break;
            }
        }
        else {
            ROS_ERROR("[%s]: check sum error cur: %d, prev: %d", ros::this_node::getName().c_str(), curCheckSum, prvCheckSum);
        }
    }
#endif

#ifndef RIC_BOARD_TEST
    bool isLizi = false, isArmadilo = false, isKomodo = false;
    std::string robotType;                                       //determent the robot type, e.g default.
    ros::param::param<bool>("komodo", isKomodo, false);
    ros::param::param<bool>("armadilo", isArmadilo, false);
    ros::param::param<bool>("lizi", isLizi, false);
    ros::param::param<std::string>("robot_type", robotType, "default");
    ros::Duration(1.0).sleep();

    if(isLizi && isArmadilo || isLizi && isKomodo || isArmadilo && isKomodo) {
        ros_utils::rosError("Two robots can't be running at the same time");
        exit(EXIT_FAILURE);
    }
    ros_utils::rosInfo("Active");
    if(isArmadilo) {
        if(robotType == "default") {
            robotican_hardware::ArmadilloRobot robot;
            robot.registerInterfaces();
            controller_manager::ControllerManager controllerManager(&robot);

            ros::AsyncSpinner asyncSpinner(1);
            asyncSpinner.start();
            while (ros::ok()) {
                robot.read();
                controllerManager.update(robot.getTime(), robot.getPeriod());
                robot.write();

            }
        }
    }
    else if(isLizi) {
        if(robotType == "default") {
            robotican_hardware::LiziRobot robot;
            robot.registerInterfaces();
            controller_manager::ControllerManager controllerManager(&robot);
            ros::AsyncSpinner asyncSpinner(1);
            asyncSpinner.start();
            ros::Rate rate(20);
            while (ros::ok()) {
                robot.read();
                controllerManager.update(robot.getTime(), robot.getPeriod());
                robot.write();
                rate.sleep();

            }
        }
    }
    else if(isKomodo) {
        if(robotType == "default") {
            robotican_hardware::KomodoRobot robot;
            robot.registerInterfaces();
            controller_manager::ControllerManager controllerManager(&robot);
            ros::AsyncSpinner asyncSpinner(1);
            asyncSpinner.start();
            ros::Rate rate(20);
            while (ros::ok()) {
                robot.read();
                controllerManager.update(robot.getTime(), robot.getPeriod());
                robot.write();
                rate.sleep();
            }
        }
    }
#endif
    return 0;
}
