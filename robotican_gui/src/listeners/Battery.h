//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Battery_H
#define ROBOTICAN_GUI_Battery_H
#include "Listener.h"
#include <std_msgs/UInt32.h>

class Battery : public Listener
{
public:

    Battery();
    int getBatteryPwr();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    int _batPower;
    void _chatterCallback(const std_msgs::UInt32::ConstPtr& msg);

};


#endif //ROBOTICAN_GUI_Battery_H
