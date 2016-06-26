//
// Created by tom on 26/06/16.
//

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;


int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_moveit_full_api");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;


    std::vector<float> targetPos, targetOrientation;

    if(!nodeHandle.getParam("target_posision",targetPos)) {
        ROS_ERROR("[%s]: target_posision parameter not found", ros::this_node::getName().c_str());
        ROS_BREAK();
    }
    else if(!nodeHandle.getParam("target_orientation",targetOrientation)) {
        ROS_ERROR("[%s]: target_orientation parameter not found", ros::this_node::getName().c_str());
        ROS_BREAK();
    }
    else if(targetPos.size() != 3) {
        ROS_ERROR("[%s]: target_posision size invalid, need to be size 3", ros::this_node::getName().c_str());
        ROS_BREAK();
    }
    else if(targetOrientation.size() != 4) {
        ROS_ERROR("[%s]: target_orientation size invalid, need to be size 4", ros::this_node::getName().c_str());
        ROS_BREAK();
    }

    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher diplay_rviz = nodeHandle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    

    GripperClient gripperClient("/gripper_controller/gripper_cmd", true);
    gripperClient.waitForServer();
    control_msgs::GripperCommandGoal openGoal;
    openGoal.command.position = 0.14;
    openGoal.command.max_effort = 0.0;
    gripperClient.sendGoal(openGoal);

    gripperClient.waitForResult();

    if(gripperClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper is now open");
    }
    else {
        ROS_ERROR("Gripper can't open");
        ROS_BREAK();
    }
    ros::Duration(10.0).sleep();

    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = targetPos[0];
    target_pose1.position.y = targetPos[1];
    target_pose1.position.z = targetPos[2];
    target_pose1.orientation.x = targetOrientation[0];
    target_pose1.orientation.y = targetOrientation[1];
    target_pose1.orientation.z = targetOrientation[2];
    target_pose1.orientation.w = targetOrientation[3];
    group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCESS":"FAILED");
    if(success) {
        group.move();
    }

    if(success) {
        ROS_INFO("Visualizing plan 1 (again)");
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        diplay_rviz.publish(display_trajectory);
// Sleep to give Rviz time to visualize the plan.

        sleep(5);
    }



    ros::Duration(10.0).sleep();
    control_msgs::GripperCommandGoal closeGoal;
    closeGoal.command.position = 0.0;
    closeGoal.command.max_effort = 0.0;
    gripperClient.sendGoal(closeGoal);

    gripperClient.waitForResult();

    if(gripperClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper is now close");
    }
    else {
        ROS_ERROR("Gripper can't close");
        ROS_BREAK();
    }


}