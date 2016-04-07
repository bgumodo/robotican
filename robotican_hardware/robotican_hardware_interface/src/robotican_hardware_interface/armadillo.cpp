//
// Created by tom on 07/04/16.
//

#include "robotican_hardware_interface/armadillo.h"

//#define DEBUG_ARMADILLO

namespace robotican_hardware {
    ArmadilloRobot::ArmadilloRobot() : RobotBase(),
                                       _positionJointInterface(),
                                       _dynamixelProController(&_jointStateInterface, &_positionJointInterface) {
        _dynamixelProController.startBroadcastingJointStates();
        std::string elevatorPubTopic, elevatorSubTopic, elevatorJointName,
                panPubTopic, panSubTopic, panJointName, tiltPubTopic, tiltSubTopic, tiltJointName;
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
#else
        if(!_nodeHandle.getParam("elevator_topic_pub", elevatorPubTopic) ||
                ! _nodeHandle.getParam("elevator_topic_sub", elevatorSubTopic) ||
                !_nodeHandle.getParam("elevator_joint", elevatorJointName)) {
            ros::shutdown();
        }
#endif

        _elevetorCmd = _nodeHandle.advertise<std_msgs::Float32>(elevatorPubTopic, 10);
        _elevetorState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::elevatorCallback, this);

        _elevatorInfo = std::pair<std::string, JointInfo_t>(elevatorJointName, JointInfo_t());
        hardware_interface::JointStateHandle jointStateHandle(_elevatorInfo.first,
                                                              &_elevatorInfo.second.position,
                                                              &_elevatorInfo.second.velocity,
                                                              &_elevatorInfo.second.effort );
        _jointStateInterface.registerHandle(jointStateHandle);

        hardware_interface::JointHandle jointHandle(_jointStateInterface.getHandle(_elevatorInfo.first), &_elevatorInfo.second.cmd);
        _positionJointInterface.registerHandle(jointHandle);





    }

    void ArmadilloRobot::elevatorCallback(const std_msgs::Float32::ConstPtr &msg) {
        _elevatorInfo.second.position = msg->data;
    }


    void ArmadilloRobot::registerInterfaces() {
        RobotBase::registerInterfaces();
        registerInterface(&_positionJointInterface);
    }

    void ArmadilloRobot::read() {
        RobotBase::read();
        _dynamixelProController.read();
    }

    void ArmadilloRobot::write() {
        RobotBase::write();
        _dynamixelProController.write();
        std_msgs::Float32 elevMsg;

        elevMsg.data = _elevatorInfo.second.cmd;
        _elevetorCmd.publish(elevMsg);

    }

    ArmadilloRobot::~ArmadilloRobot() {

    }
}

