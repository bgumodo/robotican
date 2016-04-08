//
// Created by tom on 16/02/16.
//

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <vector>


struct trajectoryPoint_t {
    trajectoryPoint_t() {
        position = 0.0;
        effort = 0.0;
        velocity = 0.0;
    }
    double position;
    double effort;
    double velocity;
};

struct feedback_t {

    trajectoryPoint_t actual;
    trajectoryPoint_t desired;
    trajectoryPoint_t error;

};

class DynamixelProActionServer {
protected:
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _jointState;
    ros::Publisher _jointCommand;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> _actionServer;
    control_msgs::FollowJointTrajectoryFeedback _feedback;
    control_msgs::FollowJointTrajectoryResult _result;
    sensor_msgs::JointState _jointInfo;
    bool _publishFeedback;

    /***********************************Methods**************************************************/
    void rosError(const char *err) {
        ROS_ERROR("[%s]: %s", ros::this_node::getName().c_str(), err);
    }

    void rosInfo(const char *info) {
        ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), info);
    }

    bool checkIfValid(std::vector<std::string> &names) {
        size_t size = names.size();
        size_t sizeInfo = _jointInfo.name.size();
        for (int i = 0; i < size; i++) {
            bool found = false;
            for (int j = 0; j < size && !found; j++) {
                if (names[i] == _jointInfo.name[j])
                    found = true;
            }
            if (!found) {
                std::string err = ("Error: " + names[i] + "not found");
                rosError(err.c_str());
                return false;
            }
        }
        return true;
    }

    feedback_t getFeedback(std::string jointName, double desiredPosition, double desiredEffort,
                           double desiredVelocity) {
        feedback_t jointFeedback = feedback_t();
        size_t size = _jointInfo.name.size();
        for (int i = 0; i < size; ++i) {
            if (jointName == _jointInfo.name[i]) {
                jointFeedback.actual.position = _jointInfo.position[i];
                jointFeedback.actual.velocity = _jointInfo.velocity[i];
                jointFeedback.actual.effort = _jointInfo.effort[i];
                jointFeedback.desired.position = desiredPosition;
                jointFeedback.desired.velocity = desiredVelocity;
                jointFeedback.desired.effort = desiredEffort;

                jointFeedback.error.position = jointFeedback.desired.position - jointFeedback.actual.position;
                jointFeedback.error.velocity = jointFeedback.desired.velocity - jointFeedback.actual.velocity;
                jointFeedback.error.effort = jointFeedback.desired.effort - jointFeedback.actual.effort;
                return jointFeedback;
            }
        }

        return jointFeedback;
    }

    void waitForExecution() {
        _publishFeedback = true;
        ros::Duration(0.1).sleep();
        _publishFeedback = false;

    }

    void clrFeedback() {
        _feedback.actual.effort.clear();
        _feedback.actual.velocities.clear();
        _feedback.actual.positions.clear();

        _feedback.error.effort.clear();
        _feedback.error.velocities.clear();
        _feedback.error.positions.clear();

        _feedback.desired.effort.clear();
        _feedback.desired.velocities.clear();
        _feedback.desired.positions.clear();
    }

    void init() {

        size_t size = _jointInfo.name.size();
        _jointInfo.position.reserve(_jointInfo.name.size());
        _jointInfo.velocity.reserve(_jointInfo.name.size());
        _jointInfo.effort.reserve(_jointInfo.name.size());
    }
    /**********************************************************************************************/

    /***********************************Callback**************************************************/
    void goalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
        std::vector<std::string> jointNames = goal->trajectory.joint_names;
        if (checkIfValid(jointNames)) {
            size_t commandSize = goal->trajectory.points.size();
            for (int i = 0; i < commandSize - 1; ++i) {
                sensor_msgs::JointState command;
                command.name = goal->trajectory.joint_names;
                command.position = goal->trajectory.points[i].positions;
                command.velocity = goal->trajectory.points[i].velocities;
                command.effort = goal->trajectory.points[i].effort;


                _jointCommand.publish(command);

                ROS_INFO_STREAM(goal->trajectory.points[i]);
                clrFeedback();
                _feedback.joint_names = jointNames;
                _feedback.desired.effort = goal->trajectory.points[i].effort;
                _feedback.desired.velocities = goal->trajectory.points[i].velocities;
                _feedback.desired.positions = goal->trajectory.points[i].positions;
                waitForExecution();

            }
            sensor_msgs::JointState command;
            command.name = goal->trajectory.joint_names;
            command.position = goal->trajectory.points[commandSize - 1].positions;

            for(int i = 0; i < command.name.size(); ++i)
            {
                command.velocity.push_back(0.2);
            }

            command.effort = goal->trajectory.points[commandSize - 1].effort;
            rosInfo("Last");
            _jointCommand.publish(command);

            clrFeedback();
            _feedback.joint_names = jointNames;
            _feedback.desired.effort = goal->trajectory.points[commandSize - 1].effort;
            _feedback.desired.velocities = goal->trajectory.points[commandSize - 1].velocities;
            _feedback.desired.positions = goal->trajectory.points[commandSize - 1].positions;
            waitForExecution();

            _result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            _result.error_string = "Cool";
            _actionServer.setSucceeded(_result);
        }
        else {
            _result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
            _result.error_string = "Not cool";
            _actionServer.setAborted(_result);
        }
    }

    void preemptCallback() {
        rosInfo("Goal got cancel");
        _actionServer.setPreempted(_result);
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
        size_t msgSize = msg->name.size(), jointInfoSize = _jointInfo.name.size(), msgSizeP = msg->position.size(),
                msgSizeV = msg->velocity.size(), msgSizeE = msg->effort.size();
        for (int i = 0; i < msgSize; i++) {
            for (int j = 0; j < jointInfoSize; j++) {
                if (msg->name[i] == _jointInfo.name[j]) {
                    if (i < msgSizeP) _jointInfo.position[j] = msg->position[i];
                    if (i < msgSizeV) _jointInfo.velocity[j] = msg->velocity[i];
                    if (i < msgSizeE) _jointInfo.effort[j] = msg->effort[i];
                }
            }
        }

        if(_publishFeedback) {
            size_t size = _feedback.joint_names.size();

            _feedback.actual.positions.clear();
            _feedback.actual.velocities.clear();
            _feedback.actual.effort.clear();

            _feedback.error.positions.clear();
            _feedback.error.velocities.clear();
            _feedback.error.effort.clear();

            for (int i = 0; i < size; ++i) {
                feedback_t singleJointFeedback = getFeedback(_feedback.joint_names[i], _feedback.desired.positions[i],
                                                             0.0,
                                                             _feedback.desired.velocities[i]);
                _feedback.actual.positions.push_back(singleJointFeedback.actual.position);
                _feedback.actual.velocities.push_back(singleJointFeedback.actual.velocity);
                _feedback.actual.effort.push_back(singleJointFeedback.actual.effort);

                _feedback.error.positions.push_back(singleJointFeedback.error.position);
                _feedback.error.velocities.push_back(singleJointFeedback.error.velocity);
                _feedback.error.effort.push_back(singleJointFeedback.error.effort);
            }
            _actionServer.publishFeedback(_feedback);
        }
    }
    /**********************************************************************************************/

public:
    DynamixelProActionServer(std::string name): _actionServer(_nodeHandle, name, boost::bind(&DynamixelProActionServer::goalCallback, this, _1) ,false) {
        _actionServer.registerPreemptCallback(boost::bind(&DynamixelProActionServer::preemptCallback, this));
        if(_nodeHandle.getParam("joint_name_list", _jointInfo.name)) {
            init();
            _publishFeedback = false;
            _jointState = _nodeHandle.subscribe("joint_states", 100, &DynamixelProActionServer::jointStateCallback,
                                                this);
            _jointCommand = _nodeHandle.advertise<sensor_msgs::JointState>("joint_commands", 10);
            _actionServer.start();
            rosInfo("Server is active");
        }
        else {
            rosError("Cloud not find joint_name_list parameter");
            ros::shutdown();
        }

    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "dynamixel_pro_action_server");
    DynamixelProActionServer actionServer(ros::this_node::getName());
    ros::spin();
    return 0;
}
