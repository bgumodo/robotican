//
// Created by sub on 13/04/16.
//


#include "Odom.h"
Odom::Odom()
{
    _signalTime = 0;
}

void Odom::_chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    _signalTime = clock();
}

void Odom::subscribe()
{
    _nHandle.param<std::string>("odom_topic",_topicName, "mobile_base_controller/odom");
    _sub = _nHandle.subscribe(_topicName, 1000, &Odom::_chatterCallback, this);
}

long int Odom::getLastSignal()
{
    return _signalTime;
}


