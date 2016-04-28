//
// Created by sub on 13/04/16.
//


#include "Gps.h"
Gps::Gps()
{
    _signalTime = 0;
}

void Gps::_chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    _signalTime = clock();
}

void Gps::subscribe()
{
    _nHandle.param<std::string>("gps_topic",_topicName, "GPS/fix");
    _sub = _nHandle.subscribe(_topicName, 1000, &Gps::_chatterCallback, this);
}

long int Gps::getLastSignal()
{
    return _signalTime;
}
