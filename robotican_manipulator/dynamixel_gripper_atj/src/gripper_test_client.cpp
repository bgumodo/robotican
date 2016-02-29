//
// Created by tom on 22/02/16.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> Client;

void doneCallback(const actionlib::SimpleClientGoalState &state, const control_msgs::GripperCommandResultConstPtr &result) {
    ROS_INFO("Finish in state [%s]", state.toString().c_str());
    ROS_INFO_STREAM(result);
    ros::shutdown();
}

void activeCallback() {
    ROS_INFO("Goal hust went active");
}

void feedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr & feedback) {
    ROS_INFO_STREAM(feedback);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gripper_server");
    Client actionClient("gripper_server", true);
    ROS_INFO("Waiting for server");
    actionClient.waitForServer();
    ROS_INFO("Sending goal");

    control_msgs::GripperCommandGoal goal;
    goal.command.max_effort = 0.5;
    goal.command.position = 0.04;
    actionClient.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);
    ros::spin();
    return 0;
}
