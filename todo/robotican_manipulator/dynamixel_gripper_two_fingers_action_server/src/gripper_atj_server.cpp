//
// Created by tom on 21/02/16.
//

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>


//#define GRIPPER_DEBUG

class DynamixelGripperServer {
protected:
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _leftFingerState;
    ros::Subscriber _rightFingerState;
    ros::Publisher _jointStatesPub;
    ros::Publisher _leftCommand;
    ros::Publisher _rightCommand;
    std::vector<std::string> _names;

    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> _actionServer;
    control_msgs::GripperCommandFeedback _feedback;
    control_msgs::GripperCommandResult _result;

    double _leftPos;
    double _rightPos;
    double _leftEffort;
    double _rightEffort;
    const double MAX_EFFORT;

    bool _publishFeedback;

    /******************************Method*************************************/
    void rosError(const char *err) {
        ROS_ERROR("[%s]: %s", ros::this_node::getName().c_str(), err);
    }

    void rosInfo(const char *info) {
        ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), info);
    }

    void init() {
        _publishFeedback = false;
        _leftEffort = _leftPos = _rightEffort = _rightPos = 0.0;
    }

    bool hadFinish(double goalInPos, double epsilon,  double effort) {
        return (fabs(_leftPos - goalInPos) <= epsilon) || _leftEffort >= effort && _rightEffort >= effort;
    }

    void waitToFinish(double goalInPos, double epsilon, double effort) {
        bool toQuit = false;
        while (ros::ok() && !_actionServer.isPreemptRequested() && !toQuit) {
            _publishFeedback = true;
            ros::Duration(0.1).sleep();
            toQuit = hadFinish(goalInPos, epsilon, effort);
        }
        _publishFeedback = false;
    }

    /*************************************************************************/

    /******************************Callback***********************************/

    void goalCallback(const control_msgs::GripperCommandGoalConstPtr &goal) {
#ifdef GRIPPER_DEBUG
        char buff[1024] = {'\0'};
        sprintf(buff, "Incomming goal { pos: %f , effort: %f }", goal->command.position, goal->command.max_effort);
        rosInfo(buff);
#endif
	if ((goal->command.position>=0)&&(goal->command.position<=0.15)){
        	double computeGap = ((goal->command.position / 2.0f) - 0.02f) / 0.09f;
       // if(fabs(computeGap) < 1.0f && goal->command.position > 0) {
            const double gap2Pos = asin(computeGap) - 0.3f;
            std_msgs::Float64 leftCommand, rightCommand;
            leftCommand.data = -gap2Pos;
            rightCommand.data = gap2Pos;
            _leftCommand.publish(leftCommand);
            _rightCommand.publish(rightCommand);
            waitToFinish(gap2Pos, 0.009, goal->command.max_effort);
            _result.position = _feedback.position;
            _result.effort = _feedback.effort;
            _result.reached_goal = true;
            _result.stalled = _feedback.stalled;
            _actionServer.setSucceeded(_result);
        }
        else {
            rosError("Invalid goal gap");
            _result.reached_goal = false;
            _actionServer.setAborted(_result, ":()");
            if(goal->command.position < 0) {
                rosError("Gap is negative");
            }
        }

    }

    void leftFingerStates(const dynamixel_msgs::JointStateConstPtr &msg) {
        _leftPos = msg->current_pos;
        _leftEffort = msg->load;
    }

    void rightFingerStates(const dynamixel_msgs::JointStateConstPtr &msg) {
        _rightPos = msg->current_pos;
        _rightEffort = msg->load;
    }

    void preemptCallback() {
        rosInfo("Goal got cancel");
        _actionServer.setPreempted(_result, ":(");
    }



    /*************************************************************************/


public:
    DynamixelGripperServer(std::string name, double maxEffort): _actionServer(_nodeHandle, name, boost::bind(&DynamixelGripperServer::goalCallback, this, _1), false),
                                                                MAX_EFFORT(maxEffort) {
        if(_nodeHandle.getParam("gripper_joints", _names)) {
            init();
            _actionServer.registerPreemptCallback(boost::bind(&DynamixelGripperServer::preemptCallback, this));
            _jointStatesPub = _nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 10);
            _leftCommand = _nodeHandle.advertise<std_msgs::Float64>("left_finger_controller/command", 10);
            _rightCommand = _nodeHandle.advertise<std_msgs::Float64>("right_finger_controller/command", 10);

            _leftFingerState = _nodeHandle.subscribe("left_finger_controller/state", 10, &DynamixelGripperServer::leftFingerStates, this);
            _rightFingerState = _nodeHandle.subscribe("right_finger_controller/state", 10, &DynamixelGripperServer::rightFingerStates, this);

            _actionServer.start();
            rosInfo("Active");
        }
        else {
            rosError("Cloud not find gripper_joints parameter");
            ros::shutdown();
        }
    }
    void run() {
        ros::Rate rate(50);
        while(ros::ok()) {
            ros::spinOnce();
            sensor_msgs::JointState msg;
            msg.name = _names;
	    msg.header.stamp = ros::Time::now();
            msg.position.push_back(_leftPos);
            msg.position.push_back(_rightPos);
            msg.effort.push_back(_leftEffort);
            msg.effort.push_back(_rightEffort);
            _jointStatesPub.publish(msg);
            if(_publishFeedback) {
                _feedback.position = 2 * (0.02f + 0.09 * sin(_leftPos + 0.35));
                _feedback.effort = _leftEffort;

                _feedback.reached_goal = false;
                _feedback.stalled = _leftEffort >= MAX_EFFORT;

                _actionServer.publishFeedback(_feedback);
            }
            rate.sleep();
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc,argv, "gripper_server");
    const double MAX_EFFORT = 0.5;
    DynamixelGripperServer server(ros::this_node::getName().c_str(), MAX_EFFORT);
    server.run();
    return 0;
}
