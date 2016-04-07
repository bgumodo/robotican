//
// Created by tom on 06/04/16.
//

#include "robotican_hardware_interface/ros_utils.h"

namespace ros_utils {

    void rosInfo(const char *info) {
        ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), info);
    }

    void rosWarn(const char *warn) {
        ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), warn);
    }

    void rosError(const char *error) {
        ROS_ERROR("[%s]: %s", ros::this_node::getName().c_str(), error);
    }
}