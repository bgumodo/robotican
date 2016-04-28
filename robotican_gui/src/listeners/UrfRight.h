//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_UrfRightListener_H
#define ROBOTICAN_GUI_UrfRightListener_H
#include "listeners/Listener.h"
#include <sensor_msgs/Range.h>

class UrfRight : public Listener
{
public:
    UrfRight();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Range::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_UrfRightListener_H
