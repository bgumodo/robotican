//
// Created by tom on 06/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ROS_UTILS_H
#define ROBOTICAN_HARDWARE_INTERFACE_ROS_UTILS_H

#include <ros/ros.h>

namespace ros_utils {

    void rosInfo(const char *info);

    void rosWarn(const char *warn);

    void rosError(const char *error);


}

#endif //ROBOTICAN_HARDWARE_INTERFACE_ROS_UTILS_H
