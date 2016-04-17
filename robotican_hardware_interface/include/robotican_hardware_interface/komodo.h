//
// Created by tom on 17/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
#define ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <robotican_hardware_interface/dynamixel_pro_controller.h>

namespace robotican_hardware {
    class KomodoRobot : public RobotBase {
    private:
        bool _haveFourMotors;
        std::pair<std::string, JointInfo_t> _rearLeftMotorJointInfo;
        std::pair<std::string, JointInfo_t> _rearRightMotorJointInfo;

        ros::Publisher _rearLeftMotorCmd;
        ros::Publisher _rearRightMotorCmd;

        ros::Subscriber _rearLeftMotorState;
        ros::Subscriber _rearRightMotorState;

        void rearRightMotorStateCallback(const ric_board::Motor::ConstPtr &msg);

        void rearLeftMotorStateCallback(const ric_board::Motor::ConstPtr &msg);

    public:
        KomodoRobot();
        virtual ~KomodoRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}


#endif //ROBOTICAN_HARDWARE_INTERFACE_KOMODO_H
