//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Gps_H
#define ROBOTICAN_GUI_Gps_H
#include "listeners/Listener.h"
#include <sensor_msgs/NavSatFix.h>

class Gps : public Listener
{
public:
    Gps();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_Gps_H
