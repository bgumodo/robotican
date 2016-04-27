//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Odom_H
#define ROBOTICAN_GUI_Odom_H
#include "listeners/Listener.h"
#include <nav_msgs/Odometry.h>

class Odom : public Listener
{
public:
    Odom();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_Odom_H
