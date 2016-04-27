
#include "RearCam.h"
RearCam::RearCam()
{
    _signalTime = 0;
}

void RearCam::_chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    _signalTime = clock();
}

void RearCam::subscribe()
{
    _nHandle.param<std::string>("RearCamTopic",_topicName, "rear_camera/image_raw");
    _sub = _nHandle.subscribe(_topicName, 1000, &RearCam::_chatterCallback, this);
}

long int RearCam::getLastSignal()
{
    return _signalTime;
}