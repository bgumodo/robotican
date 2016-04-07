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
                panPubTopic, panSubTopic, panJointName,
                tiltPubTopic, tiltSubTopic, tiltJointName;
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
                !_nodeHandle.getParam("elevator_joint", elevatorJointName) ||
                !_nodeHandle.getParam("pan_topic_pub", panPubTopic) ||
                !_nodeHandle.getParam("pan_topic_sub", panSubTopic) ||
                !_nodeHandle.getParam("pam_joint", panJointName) ||
                !_nodeHandle.getParam("tilt_topic_pub", tiltPubTopic) ||
                !_nodeHandle.getParam("tilt_topic_sub", tiltSubTopic) ||
                !_nodeHandle.getParam("tilt_joint", tiltJointName)) {
            ros::shutdown();
        }
#endif

        _elevetorCmd = _nodeHandle.advertise<std_msgs::Float32>(elevatorPubTopic, 10);
        _panCmd = _nodeHandle.advertise<std_msgs::Float32>(panPubTopic, 10);
        _tiltCmd = _nodeHandle.advertise<std_msgs::Float32>(tiltPubTopic, 10);
        _tiltCmd = _nodeHandle.advertise<std_msgs::Float32>(tiltPubTopic, 10);
        _tiltCmd = _nodeHandle.advertise<std_msgs::Float32>(tiltPubTopic, 10);

        _elevetorState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::elevatorCallback, this);
        _panState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::panCallback, this);
        _tiltState = _nodeHandle.subscribe<std_msgs::Float32>(elevatorSubTopic, 10, &ArmadilloRobot::tiltCallback, this);



        _elevatorInfo = std::pair<std::string, JointInfo_t>(elevatorJointName, JointInfo_t());
        _panInfo = std::pair<std::string, JointInfo_t>(panJointName, JointInfo_t());
        _tiltInfo = std::pair<std::string, JointInfo_t>(tiltJointName, JointInfo_t());

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
        _jointStateInterface.registerHandle(elevatorJointStateHandle);
        _jointStateInterface.registerHandle(panJointStateHandle);
        _jointStateInterface.registerHandle(tiltJointStateHandle);

        hardware_interface::JointHandle elevatorJointHandle(_jointStateInterface.getHandle(_elevatorInfo.first), &_elevatorInfo.second.cmd);
        hardware_interface::JointHandle panJointHandle(_jointStateInterface.getHandle(_panInfo.first), &_panInfo.second.cmd);
        hardware_interface::JointHandle tiltJointHandle(_jointStateInterface.getHandle(_tiltInfo.first), &_tiltInfo.second.cmd);

        _positionJointInterface.registerHandle(elevatorJointHandle);
        _positionJointInterface.registerHandle(panJointHandle);
        _positionJointInterface.registerHandle(tiltJointHandle);





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

