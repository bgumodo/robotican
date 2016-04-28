
#include "UrfLeft.h"
UrfLeft::UrfLeft()
{
    _signalTime = 0;
}

void UrfLeft::_chatterCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _signalTime = clock();
}

void UrfLeft::subscribe()
{
    _nHandle.param<std::string>("UrfLeftTopic",_topicName, "URF/left");
    _sub = _nHandle.subscribe(_topicName, 1000, &UrfLeft::_chatterCallback, this);
}

long int UrfLeft::getLastSignal()
{
    return _signalTime;
}