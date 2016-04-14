//
// Created by tom on 06/04/16.
//

#include <ros/ros.h>
#include <robotican_hardware_interface/robot_base.h>
#include <robotican_hardware_interface/armadillo.h>
#include <robotican_hardware_interface/LiziRobot.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <controller_manager/controller_manager.h>


int main(int argc, char **argv) {

    ros::init(argc, argv, "robotican_hardware_interface_node");
    ros::NodeHandle nodeHandle;
    bool isLizi = false, isArmadilo = false, isKomodo = false;
    ros::param::param("komodo", isKomodo, false);
    ros::param::param("armadilo", isArmadilo, false);
    ros::param::param("lizi", isLizi, false);

    if(isLizi && isArmadilo || isLizi && isKomodo || isArmadilo && isKomodo) {
        ros_utils::rosError("Two robots can't be running at the same time");
        exit(EXIT_FAILURE);
    }
    ros_utils::rosInfo("Active");
    if(isArmadilo) {
        robotican_hardware::ArmadilloRobot robot;
        robot.registerInterfaces();
        controller_manager::ControllerManager controllerManager(&robot);

        ros::AsyncSpinner asyncSpinner(1);
        asyncSpinner.start();
        while(ros::ok()) {
            robot.read();
            controllerManager.update(robot.getTime(), robot.getPeriod());
            robot.write();

        }
    }
    else if(isLizi) {
        robotican_hardware::LiziRobot robot;
        robot.registerInterfaces();
        controller_manager::ControllerManager controllerManager(&robot);
        ros::AsyncSpinner asyncSpinner(1);
        asyncSpinner.start();
        while(ros::ok()) {
            robot.read();
            controllerManager.update(robot.getTime(), robot.getPeriod());
            robot.write();

        }
    }
    else if(isKomodo) {
//        robotican_hardware::ArmadilloRobot robot;
//        robot.registerInterfaces();
//        controller_manager::ControllerManager controllerManager(&robot);
//        ros::AsyncSpinner asyncSpinner(1);
//        asyncSpinner.start();
//        while(ros::ok()) {
//            robot.read();
//            controllerManager.update(robot.getTime(), robot.getPeriod());
//            robot.write();
//
//        }
    }
    return 0;
}