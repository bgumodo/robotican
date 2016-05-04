//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Kinect2Listener_H
#define ROBOTICAN_GUI_Kinect2Listener_H
#include "listeners/Listener.h"
#include <sensor_msgs/Image.h>

class Kinect2 : public Listener
{
public:
    Kinect2();
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
