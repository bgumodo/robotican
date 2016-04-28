//
// Created by sub on 13/04/16.
//


#include "Arm.h"
Arm::Arm()
{
    _signalTime = 0;
}

void Arm::_chatterCallback(const control_msgs::FollowJointTrajectoryFeedback::ConstPtr& msg)
{
    _signalTime = clock();
}

void Arm::subscribe()
{
    _nHandle.param<std::string>("arm_topic",_topicName, "arm_trajectory_controller/follow_joint_trajectory/feedback");
    _sub = _nHandle.subscribe(_topicName, 1000, &Arm::_chatterCallback, this);
}

long int Arm::getLastSignal()
{
    return _signalTime;
}
