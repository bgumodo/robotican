//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_UrfRearListener_H
#define ROBOTICAN_GUI_UrfRearListener_H
#include "listeners/Listener.h"
#include <sensor_msgs/Range.h>

class UrfRear : public Listener
{
public:
    UrfRear();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Range::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_UrfRearListener_H
