//
// Created by tom on 14/04/16.
//

#include "robotican_hardware_interface/LiziRobot.h"

namespace robotican_hardware {


    void LiziRobot::rearRightMotorStateCallback(const ric_board::Motor::ConstPtr &msg) {
        _rearRightMotorJointInfo.second.position = msg->position;
        _rearRightMotorJointInfo.second.velocity = msg->velocity;
    }

    void LiziRobot::rearLeftMotorStateCallback(const ric_board::Motor::ConstPtr &msg) {
        _rearLeftMotorJointInfo.second.position = msg->position;
        _rearLeftMotorJointInfo.second.velocity = msg->velocity;
    }

    LiziRobot::LiziRobot() {
        std::string rearLeftMotorPub, rearRightMotorPub, rearLeftMotorSub,
                rearRightMotorSub, rearLeftJointName, rearRightJointName,
                panPubTopic, panSubTopic, panJointName,
                tiltPubTopic, tiltSubTopic, tiltJointName;

        if(!_nodeHandle.getParam("rear_left_motor_pub", rearLeftMotorPub) ||
           !_nodeHandle.getParam("rear_right_motor_pub", rearRightMotorPub) ||
           !_nodeHandle.getParam("rear_left_motor_joint_name", rearLeftJointName) ||
           !_nodeHandle.getParam("rear_right_motor_joint_name", rearRightJointName) ||
           !_nodeHandle.getParam("rear_right_motor_sub", rearRightMotorSub) ||
           !_nodeHandle.getParam("rear_left_motor_sub", rearLeftMotorSub)) {    /* parameters that must be instalize for the robot to work*/

            ros_utils::rosError("The robot operator need to provide parameters : 'left_motor_topic', 'right_motor_topic'"
                                        ", 'left_motor_joint_name', 'right_motor_joint_name' to run the robot");
            ros::shutdown();
        }

        _rearLeftMotorJointInfo = std::pair<std::string, JointInfo_t>(rearLeftJointName, JointInfo_t());
        _rearRightMotorJointInfo = (std::pair<std::string, JointInfo_t>(rearRightJointName, JointInfo_t()));
        _rearLeftMotorCmd = _nodeHandle.advertise<std_msgs::Float32>(rearLeftMotorPub, 10);
        _rearRightMotorCmd = _nodeHandle.advertise<std_msgs::Float32>(rearRightMotorPub, 10);
        _rearLeftMotorState = _nodeHandle.subscribe<ric_board::Motor>(rearLeftMotorSub, 10 ,&LiziRobot::rearLeftMotorStateCallback, this);
        _rearRightMotorState = _nodeHandle.subscribe<ric_board::Motor>(rearRightMotorSub, 10 ,&LiziRobot::rearRightMotorStateCallback, this);

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


        if(_nodeHandle.getParam("pan_topic_pub", panPubTopic)
           && _nodeHandle.getParam("pan_topic_sub", panSubTopic)
           && _nodeHandle.getParam("pan_joint", panJointName)
           && _nodeHandle.getParam("tilt_topic_pub", tiltPubTopic)
           && _nodeHandle.getParam("tilt_topic_sub", tiltSubTopic)
           && _nodeHandle.getParam("tilt_joint", tiltJointName)) {

            _panCmd = _nodeHandle.advertise<std_msgs::Float32>(panPubTopic, 10);
            _tiltCmd = _nodeHandle.advertise<std_msgs::Float32>(tiltPubTopic, 10);

            _panState = _nodeHandle.subscribe<std_msgs::Float32>(panSubTopic, 10, &LiziRobot::panCallback, this);
            _tiltState = _nodeHandle.subscribe<std_msgs::Float32>(tiltSubTopic, 10, &LiziRobot::tiltCallback, this);

            _panInfo = std::pair<std::string, JointInfo_t>(panJointName, JointInfo_t());
            _tiltInfo = std::pair<std::string, JointInfo_t>(tiltJointName, JointInfo_t());

            hardware_interface::JointStateHandle panJointStateHandle(_panInfo.first,
                                                                     &_panInfo.second.position,
                                                                     &_panInfo.second.velocity,
                                                                     &_panInfo.second.effort );

            hardware_interface::JointStateHandle tiltJointStateHandle(_tiltInfo.first,
                                                                      &_tiltInfo.second.position,
                                                                      &_tiltInfo.second.velocity,
                                                                      &_tiltInfo.second.effort );

            _jointStateInterface.registerHandle(panJointStateHandle);
            _jointStateInterface.registerHandle(tiltJointStateHandle);

            hardware_interface::JointHandle panJointHandle(_jointStateInterface.getHandle(_panInfo.first), &_panInfo.second.cmd);
            hardware_interface::JointHandle tiltJointHandle(_jointStateInterface.getHandle(_tiltInfo.first), &_tiltInfo.second.cmd);

            _positionJointInterface.registerHandle(panJointHandle);
            _positionJointInterface.registerHandle(tiltJointHandle);

        }
    }

    LiziRobot::~LiziRobot() {

    }

    void LiziRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        if(!_panInfo.first.empty() && _tiltInfo.first.empty()) {
            registerInterface(&_positionJointInterface);
        }
    }

    void LiziRobot::read() {
        RobotBase::read();
    }

    void LiziRobot::write() {
        RobotBase::write();

        std_msgs::Float32 leftMsg, rightMsg;

        leftMsg.data = _rearLeftMotorJointInfo.second.cmd;
        rightMsg.data = _rearRightMotorJointInfo.second.cmd;

        _rearLeftMotorCmd.publish(leftMsg);
        _rearRightMotorCmd.publish(rightMsg);

        if(!_panInfo.first.empty() && _tiltInfo.first.empty()) {
            std_msgs::Float32 panMsg, tiltMsg;

            panMsg.data = _panInfo.second.cmd;
            tiltMsg.data = _tiltInfo.second.cmd;

            _panCmd.publish(panMsg);
            _tiltCmd.publish(tiltMsg);
        }

    }

    void LiziRobot::panCallback(const std_msgs::Float32::ConstPtr &msg) {
        _panInfo.second.position = msg->data;
    }

    void LiziRobot::tiltCallback(const std_msgs::Float32::ConstPtr &msg) {
        _tiltInfo.second.position = msg->data;
    }
}