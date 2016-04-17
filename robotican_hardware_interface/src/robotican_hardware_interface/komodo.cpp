//
// Created by tom on 17/04/16.
//
#include "robotican_hardware_interface/komodo.h"

namespace robotican_hardware {

    void KomodoRobot::rearRightMotorStateCallback(const ric_board::Motor::ConstPtr &msg) {
        _rearRightMotorJointInfo.second.position = msg->position;
        _rearRightMotorJointInfo.second.velocity = msg->velocity;
    }

    void KomodoRobot::rearLeftMotorStateCallback(const ric_board::Motor::ConstPtr &msg) {
        _rearLeftMotorJointInfo.second.position = msg->position;
        _rearLeftMotorJointInfo.second.velocity = msg->velocity;
    }

    KomodoRobot::KomodoRobot() {
        _haveFourMotors = true;
        std::string rearLeftMotorPub, rearRightMotorPub, rearLeftMotorSub,
                rearRightMotorSub, rearLeftJointName, rearRightJointName;

        if(!_nodeHandle.getParam("rear_left_motor_joint_name", rearLeftJointName)
           || !_nodeHandle.getParam("rear_right_motor_joint_name", rearRightJointName)) {    /* parameters that must be instalize for the robot to work*/

            ros_utils::rosError("The robot operator need to provide parameters : "
                                        "'rear_left_motor_joint_name', 'rear_right_motor_joint_name' to run the robot");
            ros::shutdown();
        }

        if(!_nodeHandle.getParam("rear_left_motor_pub", rearLeftMotorPub)
           || !_nodeHandle.getParam("rear_right_motor_pub", rearRightMotorPub)
           || !_nodeHandle.getParam("rear_right_motor_sub", rearRightMotorSub)
           || !_nodeHandle.getParam("rear_left_motor_sub", rearLeftMotorSub)) {

            _nodeHandle.getParam("left_motor_sub", rearLeftMotorSub);
            _nodeHandle.getParam("right_motor_sub", rearRightMotorSub);
            _haveFourMotors = false;
        }

        _rearLeftMotorJointInfo = std::pair<std::string, JointInfo_t>(rearLeftJointName, JointInfo_t());
        _rearRightMotorJointInfo = (std::pair<std::string, JointInfo_t>(rearRightJointName, JointInfo_t()));

        if(_haveFourMotors) {
            _rearLeftMotorCmd = _nodeHandle.advertise<std_msgs::Float32>(rearLeftMotorPub, 10);
            _rearRightMotorCmd = _nodeHandle.advertise<std_msgs::Float32>(rearRightMotorPub, 10);
        }

        _rearLeftMotorState = _nodeHandle.subscribe<ric_board::Motor>(rearLeftMotorSub, 10 ,&KomodoRobot::rearLeftMotorStateCallback, this);
        _rearRightMotorState = _nodeHandle.subscribe<ric_board::Motor>(rearRightMotorSub, 10 ,&KomodoRobot::rearRightMotorStateCallback, this);

        hardware_interface::JointStateHandle leftJointStateHandle(_rearLeftMotorJointInfo.first,
                                                                  &_rearLeftMotorJointInfo.second.position,
                                                                  &_rearLeftMotorJointInfo.second.velocity,
                                                                  &_rearLeftMotorJointInfo.second.effort);
        hardware_interface::JointStateHandle rightJointStateHandle(_rearRightMotorJointInfo.first,
                                                                   &_rearRightMotorJointInfo.second.position,
                                                                   &_rearRightMotorJointInfo.second.velocity,
                                                                   &_rearRightMotorJointInfo.second.effort);

        _jointStateInterface.registerHandle(leftJointStateHandle);
        _jointStateInterface.registerHandle(rightJointStateHandle);

        hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_rearLeftMotorJointInfo.first),
                                                        &_rearLeftMotorJointInfo.second.cmd);
        hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rearRightMotorJointInfo.first),
                                                         &_rearRightMotorJointInfo.second.cmd);

        _velocityJointInterface.registerHandle(leftJointHandle);
        _velocityJointInterface.registerHandle(rightJointHandle);

    }

    KomodoRobot::~KomodoRobot() {

    }

    void KomodoRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
    }

    void KomodoRobot::read() {
        RobotBase::read();
    }

    void KomodoRobot::write() {
        RobotBase::write();
        if(_haveFourMotors) {
            std_msgs::Float32 leftMsg, rightMsg;

            leftMsg.data = _rearLeftMotorJointInfo.second.cmd;
            rightMsg.data = _rearRightMotorJointInfo.second.cmd;

            _rearLeftMotorCmd.publish(leftMsg);
            _rearRightMotorCmd.publish(rightMsg);
        }
    }
}