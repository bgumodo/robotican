//
// Created by tom on 06/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_ROBOT_BASE_H
#define ROBOTICAN_HARDWARE_INTERFACE_ROBOT_BASE_H

#include <ros/ros.h>
#include <ric_board/Motor.h>
#include <std_msgs/Float32.h>
#include <hardware_interface/robot_hw.h>
#include <robotican_hardware_interface/ros_utils.h>
#include <hardware_interface/joint_command_interface.h>
#include <robotican_hardware_interface/Device.h>

namespace robotican_hardware {
    class Device;

    struct JointInfo_t {
        double position;
        double effort;
        double velocity;
        double cmd;

        JointInfo_t() {
            position = effort = velocity = cmd = 0;
        }
    };

    class RobotBase : public hardware_interface::RobotHW {
    private:
        std::pair<std::string, JointInfo_t> _leftMotorJointInfo;
        std::pair<std::string, JointInfo_t> _rightMotorJointInfo;
        ros::Publisher _leftMotorCmd;
        ros::Publisher _rightMotorCmd;
        ros::Subscriber _leftMotorState;
        ros::Subscriber _rightMotorState;
        ros::Time _time;

        void rightMotorStateCallback(const ric_board::Motor::ConstPtr &msg);
        void leftMotorStateCallback(const ric_board::Motor::ConstPtr &msg);

    protected:
        ros::NodeHandle _nodeHandle;
        hardware_interface::JointStateInterface _jointStateInterface;
        hardware_interface::VelocityJointInterface _velocityJointInterface;

    public:
        RobotBase();
        virtual ~RobotBase();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
        ros::Time getTime();
        ros::Duration getPeriod();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_ROBOT_BASE_H
