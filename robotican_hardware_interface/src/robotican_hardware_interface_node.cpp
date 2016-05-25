//
// Created by tom on 06/04/16.
//



#include <ros/ros.h>
#include <robotican_hardware_interface/robot_base.h>
#include <robotican_hardware_interface/armadillo.h>
#include <robotican_hardware_interface/LiziRobot.h>
#include <robotican_hardware_interface/komodo.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <controller_manager/controller_manager.h>
#include <robotican_hardware_interface/RiCBoardManager.h>


//#define RIC_BOARD_TEST

int main(int argc, char **argv) {

    ros::init(argc, argv, "robotican_hardware_interface_node");
    ros::NodeHandle nodeHandle;
#ifdef RIC_BOARD_TEST
    robotican_hardware::RobotBase robot;
    robot.registerInterfaces();
    controller_manager::ControllerManager controllerManager(&robot);
    ros::AsyncSpinner asyncSpinner(1);
    asyncSpinner.start();
    ros::Rate loopRate(50);

    while (ros::ok()) {
        robot.read();
        controllerManager.update(robot.getTime(), robot.getPeriod());
        robot.write();
        loopRate.sleep();

    }
#endif

#ifndef RIC_BOARD_TEST
    bool isLizi = false, isArmadilo = false, isKomodo = false;
    std::string robotType;                                       //determent the robot type, e.g default.
    ros::param::param<bool>("komodo", isKomodo, false);
    ros::param::param<bool>("armadilo", isArmadilo, false);
    ros::param::param<bool>("lizi", isLizi, false);
    ros::param::param<std::string>("robot_type", robotType, "default");
    ros::Duration(1.0).sleep();

    if(isLizi && isArmadilo || isLizi && isKomodo || isArmadilo && isKomodo) {
        ros_utils::rosError("Two robots can't be running at the same time");
        exit(EXIT_FAILURE);
    }
    ros_utils::rosInfo("Active");

    if(isArmadilo) {
        if(robotType == "default") {
            robotican_hardware::ArmadilloRobot robot;
            robot.registerInterfaces();
            controller_manager::ControllerManager controllerManager(&robot);

            ros::AsyncSpinner asyncSpinner(1);
            asyncSpinner.start();
            while (ros::ok()) {
                robot.read();
                controllerManager.update(robot.getTime(), robot.getPeriod());
                robot.write();

            }
        }
    }
    else if(isLizi) {
        if(robotType == "default") {
            robotican_hardware::LiziRobot robot;
            robot.registerInterfaces();
            controller_manager::ControllerManager controllerManager(&robot);
            ros::AsyncSpinner asyncSpinner(1);
            asyncSpinner.start();
            ros::Rate rate(20);
            while (ros::ok()) {
                robot.read();
                controllerManager.update(robot.getTime(), robot.getPeriod());
                robot.write();
                rate.sleep();

            }
        }
    }
    else if(isKomodo) {
        if(robotType == "default") {
            robotican_hardware::KomodoRobot robot;
            robot.registerInterfaces();
            controller_manager::ControllerManager controllerManager(&robot);
            ros::AsyncSpinner asyncSpinner(1);
            asyncSpinner.start();
            ros::Rate rate(20);
            while (ros::ok()) {
                robot.read();
                controllerManager.update(robot.getTime(), robot.getPeriod());
                robot.write();
                rate.sleep();
            }
        }
    }
#endif
    return 0;
}
