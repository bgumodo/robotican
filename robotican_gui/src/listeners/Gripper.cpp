//
// Created by sub on 13/04/16.
//


#include "Gripper.h"
Gripper::Gripper()
{
    _signalTime = 0;
}

void Gripper::_chatterCallback(const control_msgs::GripperCommandFeedback::ConstPtr& msg)
{
    _signalTime = clock();
}

void Gripper::subscribe()
{
    _nHandle.param<std::string>("GripperTopic",_topicName, "GripperTopic");
    _sub = _nHandle.subscribe(_topicName, 1000, &Gripper::_chatterCallback, this);
}

long int Gripper::getLastSignal()
{
    return _signalTime;
}
