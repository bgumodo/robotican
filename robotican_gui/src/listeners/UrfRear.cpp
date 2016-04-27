
#include "UrfRear.h"
UrfRear::UrfRear()
{
    _signalTime = 0;
}

void UrfRear::_chatterCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    _signalTime = clock();
}

void UrfRear::subscribe()
{
    _nHandle.param<std::string>("UrfRearTopic",_topicName, "URF/rear");
    _sub = _nHandle.subscribe(_topicName, 1000, &UrfRear::_chatterCallback, this);
}

long int UrfRear::getLastSignal()
{
    return _signalTime;
}