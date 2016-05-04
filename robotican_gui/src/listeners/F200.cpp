
#include "F200.h"
F200::F200()
{
    _signalTime = 0;
}

void F200::_chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    _signalTime = clock();
}

void F200::subscribe()
{
    _nHandle.param<std::string>("front_cam_topic",_topicName, "f200/depth/image_raw");
    _sub = _nHandle.subscribe(_topicName, 1000, &F200::_chatterCallback, this);
}

long int F200::getLastSignal()
{
    return _signalTime;
}