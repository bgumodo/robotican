//
// Created by tom on 07/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H
#define ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <dynamixel_pro_controller/dynamixel_pro_controller.h>

namespace robotican_hardware {
    class ArmadilloRobot : public RobotBase {
    protected:
        hardware_interface::PositionJointInterface _positionJointInterface;
        hardware_interface::PosVelJointInterface _posVelJointInterface;

    private:
        bool _first;
        dynamixel_pro_controller::DynamixelProController _dynamixelProController;
        std::pair<std::string, JointInfo_t> _elevatorInfo;
        std::pair<std::string, JointInfo_t> _panInfo;
        std::pair<std::string, JointInfo_t> _tiltInfo;
        std::pair<std::string, JointInfo_t> _leftFingerInfo;
        std::pair<std::string, JointInfo_t> _rightFingerInfo;

        ros::Publisher _elevetorCmd;
        ros::Publisher _panCmd;
        ros::Publisher _tiltCmd;
        ros::Publisher _leftFingerCmd;
        ros::Publisher _rightFingerCmd;

        ros::Subscriber _elevetorState;
        ros::Subscriber _panState;
        ros::Subscriber _tiltState;
        ros::Subscriber _leftFingerState;
        ros::Subscriber _rightFingerState;

        void elevatorCallback(const std_msgs::Float32::ConstPtr &msg);
        void panCallback(const std_msgs::Float32::ConstPtr &msg);
        void tiltCallback(const std_msgs::Float32::ConstPtr &msg);
        void leftFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg);
        void rightFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg);


    public:
        ArmadilloRobot();
        virtual ~ArmadilloRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_ARMADILLO_H
