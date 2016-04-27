//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Gripper_H
#define ROBOTICAN_GUI_Gripper_H
#include "listeners/Listener.h"
#include <control_msgs/GripperCommandFeedback.h>

class Gripper : public Listener
{
public:
    Gripper();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const control_msgs::GripperCommandFeedback::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_Gripper_H
