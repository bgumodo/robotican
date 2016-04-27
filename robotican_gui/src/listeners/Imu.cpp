//
// Created by sub on 13/04/16.
//


#include "Imu.h"
Imu::Imu()
{
    _signalTime = 0;
}

void Imu::_chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    _signalTime = clock();
}

void Imu::subscribe()
{
    _nHandle.param<std::string>("ImuTopic",_topicName, "ImuTopic");
    _sub = _nHandle.subscribe(_topicName, 1000, &Imu::_chatterCallback, this);
}

long int Imu::getLastSignal()
{
    return _signalTime;
}
