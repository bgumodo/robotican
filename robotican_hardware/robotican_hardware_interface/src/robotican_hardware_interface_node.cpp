//
// Created by tom on 06/04/16.
//

#include <ros/ros.h>
#include <robotican_hardware_interface/robot_base.h>
#include <robotican_hardware_interface/armadillo.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotican_hardware_interface_node");
//    robotican_hardware::RobotBase base;
    robotican_hardware::ArmadilloRobot robot;
    controller_manager::ControllerManager controllerManager(&robot);

    ros_utils::rosInfo("Active");
    ros::AsyncSpinner asyncSpinner(1);
    asyncSpinner.start();
    while(ros::ok()) {
        robot.read();
        controllerManager.update(robot.getTime(), robot.getPeriod());
        robot.write();
    }
    return 0;
}