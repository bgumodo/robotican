//
// Created by sub on 13/04/16.
//


#include "Arm.h"
Arm::Arm()
{
    _signalTime = 0;
}

void Arm::_chatterCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    _signalTime = clock();
}

void Arm::subscribe()
{
    _nHandle.param<std::string>("arm_topic",_topicName, "arm_trajectory_controller/state");
    _sub = _nHandle.subscribe(_topicName, 1000, &Arm::_chatterCallback, this);
}

long int Arm::getLastSignal()
{
    return _signalTime;
}
