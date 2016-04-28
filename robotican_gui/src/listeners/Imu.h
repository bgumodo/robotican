//
// Created by sub on 13/04/16.
//

#ifndef ROBOTICAN_GUI_Imu_H
#define ROBOTICAN_GUI_Imu_H
#include "listeners/Listener.h"
#include <sensor_msgs/Imu.h>

class Imu : public Listener
{
public:
    Imu();
    long int getLastSignal();
    void subscribe();

private:

    ros::NodeHandle _nHandle;
    ros::Subscriber _sub;
    std::string _topicName;
    long int _signalTime;

    void _chatterCallback(const sensor_msgs::Imu::ConstPtr& msg);
};


#endif //ROBOTICAN_GUI_Imu_H
