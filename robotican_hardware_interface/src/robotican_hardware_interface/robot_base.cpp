//
// Created by tom on 06/04/16.
//

#include "robotican_hardware_interface/robot_base.h"

//#define DEBUG_BASE_ROBOT

namespace robotican_hardware {
    RobotBase::RobotBase() {
        _time = ros::Time::now();
        _boardManager.connect();
        ros::Rate loopRate(50);
        while (_boardManager.getConnectState() != ConnectEnum::Connected) { loopRate.sleep(); }
        _boardManager.buildDevices();
        _boardManager.buildDevices(&_jointStateInterface, &_velocityJointInterface);
    }

    void RobotBase::registerInterfaces() {
        registerInterface(&_jointStateInterface);
        registerInterface(&_velocityJointInterface);
    }

    ros::Time RobotBase::getTime() {
        return ros::Time::now();
    }

    ros::Duration RobotBase::getPeriod() {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - _time;
        _time = now;
        return period;
    }

    void RobotBase::read() { }

    void RobotBase::write() {
        _boardManager.write();
    }
    RobotBase::~RobotBase() {
        _boardManager.disconnect();
    }

}