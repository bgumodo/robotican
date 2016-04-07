//
// Created by tom on 07/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H
#define ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <dynamixel_pro_controller/dynamixel_pro_controller.h>

namespace robotican_hardware {
    class ArmadilloRobot : public RobotBase {

    private:
        dynamixel_pro_controller::DynamixelProController _dynamixelProController;
        std::pair<std::string, JointInfo_t> _elevatorInfo;
        ros::Publisher _elevetorCmd;
        ros::Subscriber _elevetorState;

        void elevatorCallback(const std_msgs::Float32::ConstPtr &msg);

    protected:
        hardware_interface::PositionJointInterface _positionJointInterface;

    public:
        ArmadilloRobot();
        virtual ~ArmadilloRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H
