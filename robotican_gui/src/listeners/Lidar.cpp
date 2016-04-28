//
// Created by sub on 13/04/16.
//

#include "Lidar.h"

Lidar::Lidar()
{
    _signalTime = 0;
}

void Lidar::_chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    _signalTime = clock();
}

void Lidar::subscribe()
{
    _nHandle.param<std::string>("lidar_topic",_topicName, "scan");
    _sub = _nHandle.subscribe(_topicName, 1000, &Lidar::_chatterCallback, this);
}

long int Lidar::getLastSignal()
{
    return _signalTime;
}
