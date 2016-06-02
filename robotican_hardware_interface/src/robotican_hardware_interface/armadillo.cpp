//
// Created by tom on 07/04/16.
//

#include "robotican_hardware_interface/armadillo.h"

//#define DEBUG_ARMADILLO

namespace robotican_hardware {
    ArmadilloRobot::ArmadilloRobot() : RobotBase(),
                                       _positionJointInterface(),_posVelJointInterface(),
                                       _dynamixelProController(&_jointStateInterface, &_posVelJointInterface) {
        _dynamixelProController.startBroadcastingJointStates();
        std::string  leftFingerPubTopic, leftFingerSubTopic,
                leftFingerJointName,
                rightFingerPubTopic, rightFingerSubTopic, rightFingerJointName;

        _boardManager.buildDevices(&_jointStateInterface, &_positionJointInterface);
        if(!_nodeHandle.getParam("left_finger_topic_pub", leftFingerPubTopic) ||
                !_nodeHandle.getParam("left_finger_topic_sub", leftFingerSubTopic) ||
                !_nodeHandle.getParam("left_finger_joint", leftFingerJointName) ||
                !_nodeHandle.getParam("right_finger_topic_pub", rightFingerPubTopic) ||
                !_nodeHandle.getParam("right_finger_topic_sub", rightFingerSubTopic) ||
                !_nodeHandle.getParam("right_finger_joint", rightFingerJointName)) {                        /* parameters that must be instalize for the robot to work*/
            ros::shutdown();
        }
        _first = true;
        _leftFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(leftFingerPubTopic, 10);
        _rightFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(rightFingerPubTopic, 10);

        _leftFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(leftFingerSubTopic, 10, &ArmadilloRobot::leftFingerCallback, this);
        _rightFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(rightFingerSubTopic, 10, &ArmadilloRobot::rightFingerCallback, this);

        _leftFingerInfo = std::pair<std::string, JointInfo_t>(leftFingerJointName, JointInfo_t());
        _rightFingerInfo = std::pair<std::string, JointInfo_t>(rightFingerJointName, JointInfo_t());

        hardware_interface::JointStateHandle leftJointStateHandle(_leftFingerInfo.first,
                                                                      &_leftFingerInfo.second.position,
                                                                      &_leftFingerInfo.second.velocity,
                                                                      &_leftFingerInfo.second.effort );

        hardware_interface::JointStateHandle rightJointStateHandle(_rightFingerInfo.first,
                                                                      &_rightFingerInfo.second.position,
                                                                      &_rightFingerInfo.second.velocity,
                                                                      &_rightFingerInfo.second.effort );

        _jointStateInterface.registerHandle(leftJointStateHandle);
        _jointStateInterface.registerHandle(rightJointStateHandle);

        hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_leftFingerInfo.first), &_leftFingerInfo.second.cmd);
        hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rightFingerInfo.first), &_rightFingerInfo.second.cmd);

        _positionJointInterface.registerHandle(leftJointHandle);
        _positionJointInterface.registerHandle(rightJointHandle);





    }

    void ArmadilloRobot::elevatorCallback(const std_msgs::Float32::ConstPtr &msg) {
        _elevatorInfo.second.position = msg->data;
    }

    void ArmadilloRobot::panCallback(const std_msgs::Float32::ConstPtr &msg) {
        _panInfo.second.position = msg->data;
    }

    void ArmadilloRobot::tiltCallback(const std_msgs::Float32::ConstPtr &msg) {
        _tiltInfo.second.position = msg->data;
    }

    void ArmadilloRobot::leftFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _leftFingerInfo.second.position = msg->current_pos;
        _leftFingerInfo.second.velocity = msg->velocity;
        _leftFingerInfo.second.effort = msg->load;
    }

    void ArmadilloRobot::rightFingerCallback(const dynamixel_msgs::JointState::ConstPtr &msg) {
        _rightFingerInfo.second.position = msg->current_pos;
        _rightFingerInfo.second.velocity = msg->velocity;
        _rightFingerInfo.second.effort = msg->load;
    }

    void ArmadilloRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        registerInterface(&_positionJointInterface);
        registerInterface(&_posVelJointInterface);
    }

    void ArmadilloRobot::read() {
        RobotBase::read();
        _dynamixelProController.read();
        if(_first) {
            _first = false;
            _leftFingerInfo.second.cmd = _leftFingerInfo.second.position;
            _rightFingerInfo.second.cmd = _rightFingerInfo.second.position;
        }
    }

    void ArmadilloRobot::write() {
        RobotBase::write();
        _dynamixelProController.write();
        std_msgs::Float64 leftMsg, rightMsg;

        leftMsg.data = _leftFingerInfo.second.cmd;
        rightMsg.data = _rightFingerInfo.second.cmd;

        _leftFingerCmd.publish(leftMsg);
        _rightFingerCmd.publish(rightMsg);

    }

    ArmadilloRobot::~ArmadilloRobot() {

    }
}

