//
// Created by tom on 20/04/16.
//

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/semantic_world/semantic_world.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

class MoveitNode {
private:
    moveit::planning_interface::MoveGroup _group;
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _moveSub;
    bool _isFinish;
    void onMove(const geometry_msgs::PoseStamped::ConstPtr &poseStamped) {
        if(_isFinish) {
            _isFinish = false;
            _group.setPoseTarget(poseStamped->pose);
            moveit::planning_interface::MoveGroup::Plan my_plan;
            bool success = _group.plan(my_plan);
            if (success) {
                _group.move();
            }
            _isFinish = true;
        }
        else {
            ROS_ERROR("[%s]: Moveit not finish with the previous mission", ros::this_node::getName().c_str());
        }
    }
public:
    MoveitNode() :  _group("arm") {
        _isFinish = true;
        std::string topicName;
        ros::param::param<std::string>("moveit_topic_name", topicName, "move_arm");
        _moveSub = _nodeHandle.subscribe(topicName, 10, &MoveitNode::onMove, this);
    }

    void run() {
        ros::spin();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv,"simple_move_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nodeHandle;

    MoveitNode moveitNode;
    moveitNode.run();
    return 0;
}