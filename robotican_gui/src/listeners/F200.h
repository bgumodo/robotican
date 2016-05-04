//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_F200Listener_H
#define ROBOTICAN_GUI_F200Listener_H
#include "listeners/Listener.h"
#include <sensor_msgs/Image.h>

class F200 : public Listener
{
public:
    F200();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Image::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_FrontCamListener_H
