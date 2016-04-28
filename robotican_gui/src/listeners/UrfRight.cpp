
#include "UrfRight.h"
UrfRight::UrfRight()
{
    _signalTime = 0;
}

void UrfRight::_chatterCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _signalTime = clock();
}

void UrfRight::subscribe()
{
    _nHandle.param<std::string>("urf_right_topic",_topicName, "URF/right");
    _sub = _nHandle.subscribe(_topicName, 1000, &UrfRight::_chatterCallback, this);
}

long int UrfRight::getLastSignal()
{
    return _signalTime;
}