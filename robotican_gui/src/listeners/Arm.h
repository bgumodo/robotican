//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Arm_H
#define ROBOTICAN_GUI_Arm_H
#include "listeners/Listener.h"
#include <control_msgs/FollowJointTrajectoryFeedback.h>

class Arm : public Listener
{
public:
    Arm();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const control_msgs::FollowJointTrajectoryFeedback::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_Arm_H
