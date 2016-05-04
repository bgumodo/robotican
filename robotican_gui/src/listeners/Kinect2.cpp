
#include "Kinect2.h"
Kinect2::Kinect2()
{
    _signalTime = 0;
}

void Kinect2::_chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    _signalTime = clock();
}

void Kinect2::subscribe()
{
    _nHandle.param<std::string>("front_cam_topic",_topicName, "kinect2/hd/image_depth_rect");
    _sub = _nHandle.subscribe(_topicName, 1000, &Kinect2::_chatterCallback, this);
}

long int Kinect2::getLastSignal()
{
    return _signalTime;
}