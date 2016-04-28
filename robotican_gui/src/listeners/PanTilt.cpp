
#include "PanTilt.h"
PanTilt::PanTilt()
{
    _signalTime = 0;
}

void PanTilt::_chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    _signalTime = clock();
}

void PanTilt::subscribe()
{
    _nHandle.param<std::string>("panTiltTopic",_topicName, "pan_tilt_controller/command");
    _sub = _nHandle.subscribe(_topicName, 1000, &PanTilt::_chatterCallback, this);
}

long int PanTilt::getLastSignal()
{
    return _signalTime;
}