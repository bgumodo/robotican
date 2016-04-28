//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI__H
#define ROBOTICAN_GUI__H

#include "listeners/Listener.h"
#include <sensor_msgs/LaserScan.h>

class Lidar : public Listener
{
public:
    Lidar();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif