//
// Created by sub on 13/04/16.
//


#include "Gripper.h"
Gripper::Gripper()
{
    _signalTime = 0;
}

void Gripper::_chatterCallback(const actionlib_msgs::GoalStatusArray& msg)
{
    _signalTime = clock();
}

void Gripper::subscribe()
{
    _nHandle.param<std::string>("gripper_topic",_topicName, "gripper_controller/gripper_cmd/status");
    _sub = _nHandle.subscribe(_topicName, 1000, &Gripper::_chatterCallback, this);
}

long int Gripper::getLastSignal()
{
    return _signalTime;
}
