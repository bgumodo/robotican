//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_PanTiltListener_H
#define ROBOTICAN_GUI_PanTiltListener_H
#include "listeners/Listener.h"
#include <std_msgs/Float64MultiArray.h>

class PanTilt : public Listener
{
public:
    PanTilt();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_PanTiltListener_H
