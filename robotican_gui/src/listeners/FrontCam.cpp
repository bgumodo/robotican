
#include "FrontCam.h"
FrontCam::FrontCam()
{
    _signalTime = 0;
}

void FrontCam::_chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    _signalTime = clock();
}

void FrontCam::subscribe()
{
    _nHandle.param<std::string>("front_cam_topic",_topicName, "front_camera/image_raw");
    _sub = _nHandle.subscribe(_topicName, 1000, &FrontCam::_chatterCallback, this);
}

long int FrontCam::getLastSignal()
{
    return _signalTime;
}