//
// Created by tom on 06/04/16.
//

#include "robotican_hardware_interface/robot_base.h"

//#define DEBUG_BASE_ROBOT

namespace robotican_hardware {
    RobotBase::RobotBase() {
        _time = ros::Time::now();
        std::string leftMotorPub, rightMotorPub, leftMotorSub, rightMotorSub ,leftJointName, rightJointName;

#ifdef DEBUG_BASE_ROBOT
        ros::param::param<std::string>("left_motor_pub", leftMotorPub, "left_motor/command");
        ros::param::param<std::string>("left_motor_sub", leftMotorSub, "left_motor/feedback");
        ros::param::param<std::string>("right_motor_pub", rightMotorPub, "right_motor/command");
        ros::param::param<std::string>("right_motor_sub", rightMotorSub, "right_motor/feedback");
        ros::param::param<std::string>("left_motor_joint_name", leftJointName, "left_wheel_joint");
        ros::param::param<std::string>("right_motor_joint_name", rightJointName, "right_wheel_joint");
#else
        if(!_nodeHandle.getParam("left_motor_pub", leftMotorPub) ||
           !_nodeHandle.getParam("right_motor_pub", rightMotorPub) ||
           !_nodeHandle.getParam("left_motor_joint_name", leftJointName) ||
           !_nodeHandle.getParam("right_motor_joint_name", rightJointName) ||
           !_nodeHandle.getParam("left_motor_sub", leftMotorSub) ||
           !_nodeHandle.getParam("right_motor_sub", rightMotorSub)) {

            ros_utils::rosError("The robot operator need to provide parameters : 'left_motor_topic', 'right_motor_topic'"
                                        ", 'left_motor_joint_name', 'right_motor_joint_name' to run the robot");
            ros::shutdown();
        }
#endif

        _leftMotorJointInfo = std::pair<std::string, JointInfo_t>(leftJointName, JointInfo_t());
        _rightMotorJointInfo = (std::pair<std::string, JointInfo_t>(rightJointName, JointInfo_t()));
        _leftMotorCmd = _nodeHandle.advertise<std_msgs::Float32>(leftMotorPub, 10);
        _rightMotorCmd = _nodeHandle.advertise<std_msgs::Float32>(rightMotorPub, 10);
        _leftMotorState = _nodeHandle.subscribe<ric_board::Motor>(leftMotorSub, 10 ,&RobotBase::leftMotorStateCallback, this);
        _rightMotorState = _nodeHandle.subscribe<ric_board::Motor>(leftMotorSub, 10 ,&RobotBase::rightMotorStateCallback, this);

        hardware_interface::JointStateHandle leftJointStateHandle(_leftMotorJointInfo.first,
                                                                  &_leftMotorJointInfo.second.position,
                                                                  &_leftMotorJointInfo.second.velocity,
                                                                  &_leftMotorJointInfo.second.effort);
        hardware_interface::JointStateHandle rightJointStateHandle(_rightMotorJointInfo.first,
                                                                   &_rightMotorJointInfo.second.position,
                                                                   &_rightMotorJointInfo.second.velocity,
                                                                   &_rightMotorJointInfo.second.effort);

        _jointStateInterface.registerHandle(leftJointStateHandle);
        _jointStateInterface.registerHandle(rightJointStateHandle);

        hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_leftMotorJointInfo.first),
                                                        &_leftMotorJointInfo.second.cmd);
        hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rightMotorJointInfo.first),
                                                         &_rightMotorJointInfo.second.cmd);

        _velocityJointInterface.registerHandle(leftJointHandle);
        _velocityJointInterface.registerHandle(rightJointHandle);

    }

    void RobotBase::leftMotorStateCallback(const ric_board::Motor::ConstPtr &msg) {
        _leftMotorJointInfo.second.position = msg->position;
        _leftMotorJointInfo.second.velocity = msg->velocity;
    }

    void RobotBase::rightMotorStateCallback(const ric_board::Motor::ConstPtr &msg) {
        _rightMotorJointInfo.second.position = msg->position;
        _rightMotorJointInfo.second.velocity = msg->velocity;
    }

    void RobotBase::registerInterfaces() {
        registerInterface(&_jointStateInterface);
        registerInterface(&_velocityJointInterface);
    }

    ros::Time RobotBase::getTime() {
        return ros::Time::now();
    }

    ros::Duration RobotBase::getPeriod() {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - _time;
        _time = now;
        return period;
    }

    void RobotBase::read() { }

    void RobotBase::write() {
        std_msgs::Float32 leftMsg, rightMsg;

        leftMsg.data = _leftMotorJointInfo.second.cmd;
        rightMsg.data = _rightMotorJointInfo.second.cmd;

        _leftMotorCmd.publish(leftMsg);
        _rightMotorCmd.publish(rightMsg);
    }
    RobotBase::~RobotBase() {

    }

}