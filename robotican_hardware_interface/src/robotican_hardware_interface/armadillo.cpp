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
        std::string elevatorPubTopic, elevatorSubTopic, elevatorJointName,
                panPubTopic, panSubTopic, panJointName,
                tiltPubTopic, tiltSubTopic, tiltJointName,
                leftFingerPubTopic, leftFingerSubTopic, leftFingerJointName,
                rightFingerPubTopic, rightFingerSubTopic, rightFingerJointName;

#ifdef DEBUG_ARMADILLO
        ros::param::param<std::string>("elevator_topic_pub", elevatorPubTopic, "left_motor/command");
        ros::param::param<std::string>("elevator_topic_sub", elevatorSubTopic, "left_motor/feedback");
        ros::param::param<std::string>("elevator_joint", elevatorJointName, "elevator_joint");

        ros::param::param<std::string>("pan_topic_pub", panPubTopic, "left_motor/command");
        ros::param::param<std::string>("pan_topic_sub", panSubTopic, "left_motor/feedback");
        ros::param::param<std::string>("pam_joint", panJointName, "elevator_joint");

        ros::param::param<std::string>("tilt_topic_pub", tiltPubTopic, "left_motor/command");
        ros::param::param<std::string>("tilt_topic_sub", tiltSubTopic, "left_motor/feedback");
        ros::param::param<std::string>("tilt_joint", tiltJointName, "elevator_joint");

        ros::param::param<std::string>("left_finger_topic_pub", leftFingerPubTopic, "elevator_joint");
        ros::param::param<std::string>("left_finger_topic_sub", leftFingerSubTopic, "elevator_joint");
        ros::param::param<std::string>("left_finger_joint", leftFingerJointName, "elevator_joint");

        ros::param::param<std::string>("right_finger_topic_pub", rightFingerPubTopic, "elevator_joint");
        ros::param::param<std::string>("right_finger_topic_sub", rightFingerSubTopic, "elevator_joint");
        ros::param::param<std::string>("right_finger_joint", rightFingerJointName, "elevator_joint");
#else
        if(!_nodeHandle.getParam("elevator_topic_pub", elevatorPubTopic) ||
                ! _nodeHandle.getParam("elevator_topic_sub", elevatorSubTopic) ||
                !_nodeHandle.getParam("elevator_joint", elevatorJointName) ||
                !_nodeHandle.getParam("pan_topic_pub", panPubTopic) ||
                !_nodeHandle.getParam("pan_topic_sub", panSubTopic) ||
                !_nodeHandle.getParam("pam_joint", panJointName) ||
                !_nodeHandle.getParam("tilt_topic_pub", tiltPubTopic) ||
                !_nodeHandle.getParam("tilt_topic_sub", tiltSubTopic) ||
                !_nodeHandle.getParam("tilt_joint", tiltJointName) ||
                !_nodeHandle.getParam("left_finger_topic_pub", leftFingerPubTopic) ||
                !_nodeHandle.getParam("left_finger_topic_sub", leftFingerSubTopic) ||
                !_nodeHandle.getParam("left_finger_joint", leftFingerJointName) ||
                !_nodeHandle.getParam("right_finger_topic_pub", rightFingerPubTopic) ||
                !_nodeHandle.getParam("right_finger_topic_sub", rightFingerSubTopic) ||
                !_nodeHandle.getParam("right_finger_joint", rightFingerJointName)) {
            ros::shutdown();
        }
#endif
        _first = true;
        _elevetorCmd = _nodeHandle.advertise<std_msgs::Float32>(elevatorPubTopic, 10);
        _panCmd = _nodeHandle.advertise<std_msgs::Float32>(panPubTopic, 10);
        _tiltCmd = _nodeHandle.advertise<std_msgs::Float32>(tiltPubTopic, 10);
        _leftFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(leftFingerPubTopic, 10);
        _rightFingerCmd = _nodeHandle.advertise<std_msgs::Float64>(rightFingerPubTopic, 10);

        _elevetorState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::elevatorCallback, this);
        _panState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::panCallback, this);
        _tiltState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::tiltCallback, this);

        _leftFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(leftFingerSubTopic, 10, &ArmadilloRobot::leftFingerCallback, this);
        _rightFingerState = _nodeHandle.subscribe<dynamixel_msgs::JointState>(rightFingerSubTopic, 10, &ArmadilloRobot::rightFingerCallback, this);

        _elevatorInfo = std::pair<std::string, JointInfo_t>(elevatorJointName, JointInfo_t());
        _panInfo = std::pair<std::string, JointInfo_t>(panJointName, JointInfo_t());
        _tiltInfo = std::pair<std::string, JointInfo_t>(tiltJointName, JointInfo_t());

        _leftFingerInfo = std::pair<std::string, JointInfo_t>(leftFingerJointName, JointInfo_t());
        _rightFingerInfo = std::pair<std::string, JointInfo_t>(rightFingerJointName, JointInfo_t());

        hardware_interface::JointStateHandle elevatorJointStateHandle(_elevatorInfo.first,
                                                                      &_elevatorInfo.second.position,
                                                                      &_elevatorInfo.second.velocity,
                                                                      &_elevatorInfo.second.effort );

        hardware_interface::JointStateHandle panJointStateHandle(_panInfo.first,
                                                                      &_panInfo.second.position,
                                                                      &_panInfo.second.velocity,
                                                                      &_panInfo.second.effort );

        hardware_interface::JointStateHandle tiltJointStateHandle(_tiltInfo.first,
                                                                      &_tiltInfo.second.position,
                                                                      &_tiltInfo.second.velocity,
                                                                      &_tiltInfo.second.effort );

        hardware_interface::JointStateHandle leftJointStateHandle(_leftFingerInfo.first,
                                                                      &_leftFingerInfo.second.position,
                                                                      &_leftFingerInfo.second.velocity,
                                                                      &_leftFingerInfo.second.effort );

        hardware_interface::JointStateHandle rightJointStateHandle(_rightFingerInfo.first,
                                                                      &_rightFingerInfo.second.position,
                                                                      &_rightFingerInfo.second.velocity,
                                                                      &_rightFingerInfo.second.effort );

        _jointStateInterface.registerHandle(elevatorJointStateHandle);
        _jointStateInterface.registerHandle(panJointStateHandle);
        _jointStateInterface.registerHandle(tiltJointStateHandle);
        _jointStateInterface.registerHandle(leftJointStateHandle);
        _jointStateInterface.registerHandle(rightJointStateHandle);

        hardware_interface::JointHandle elevatorJointHandle(_jointStateInterface.getHandle(_elevatorInfo.first), &_elevatorInfo.second.cmd);
        hardware_interface::JointHandle panJointHandle(_jointStateInterface.getHandle(_panInfo.first), &_panInfo.second.cmd);
        hardware_interface::JointHandle tiltJointHandle(_jointStateInterface.getHandle(_tiltInfo.first), &_tiltInfo.second.cmd);
        hardware_interface::JointHandle leftJointHandle(_jointStateInterface.getHandle(_leftFingerInfo.first), &_leftFingerInfo.second.cmd);
        hardware_interface::JointHandle rightJointHandle(_jointStateInterface.getHandle(_rightFingerInfo.first), &_rightFingerInfo.second.cmd);

        _positionJointInterface.registerHandle(elevatorJointHandle);
        _positionJointInterface.registerHandle(panJointHandle);
        _positionJointInterface.registerHandle(tiltJointHandle);
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
    }

    void ArmadilloRobot::read() {
        RobotBase::read();
        _dynamixelProController.read();
        if(_first) {
            _first = false;
            _elevatorInfo.second.cmd = _elevatorInfo.second.position;
            _panInfo.second.cmd = _panInfo.second.position;
            _tiltInfo.second.cmd = _tiltInfo.second.position;
            _leftFingerInfo.second.cmd = _leftFingerInfo.second.position;
            _rightFingerInfo.second.cmd = _rightFingerInfo.second.position;
        }
    }

    void ArmadilloRobot::write() {
        RobotBase::write();
        _dynamixelProController.write();
        std_msgs::Float32 elevMsg, tiltMsg, panMsg;
        std_msgs::Float64 leftMsg, rightMsg;

        elevMsg.data = _elevatorInfo.second.cmd;
        tiltMsg.data = _panInfo.second.cmd;
        panMsg.data = _tiltInfo.second.cmd;
        leftMsg.data = _leftFingerInfo.second.cmd;
        rightMsg.data = _rightFingerInfo.second.cmd;

        _elevetorCmd.publish(elevMsg);
        _panCmd.publish(tiltMsg);
        _tiltCmd.publish(panMsg);
        _leftFingerCmd.publish(leftMsg);
        _rightFingerCmd.publish(rightMsg);

    }

    ArmadilloRobot::~ArmadilloRobot() {

    }
}

