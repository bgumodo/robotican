//
// Created by tom on 22/06/16.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robotican_common/searchForColor.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class DemoNavNode {
private:
    ros::NodeHandle _nodeHandle;
    ros::ServiceClient _searchForColorClient;
    ros::Subscriber _setColor;
    MoveBaseClient _moveBaseClient;

    std::string _color;
    bool _gotMission;

    void onSetColor(const std_msgs::String::ConstPtr &msg) {

        std::string prevColor = _color;
        _color = msg->data;
        _gotMission = true;
        if (_color != prevColor)
            ROS_INFO("[%s]: color set to: %s", ros::this_node::getName().c_str(), _color.c_str());

    }

public:
    DemoNavNode() : _nodeHandle(), _moveBaseClient("move_base", true) {
        std::string serviceName = "";
        _gotMission = false;
        if (_nodeHandle.getParam("search_for_color_topic", serviceName)) {

            _searchForColorClient = _nodeHandle.serviceClient<robotican_common::searchForColor>(serviceName);
            _setColor = _nodeHandle.subscribe<std_msgs::String>("set_color", 10, &DemoNavNode::onSetColor, this);

            while (!_searchForColorClient.waitForExistence(ros::Duration(5.0)))
                ROS_WARN("[%s]: Waiting for the %s server to come up", ros::this_node::getName().c_str(),
                         serviceName.c_str());

            while (!_moveBaseClient.waitForServer(ros::Duration(5.0)))
                ROS_WARN("[%s]: Waiting for the move_base action server to come up", ros::this_node::getName().c_str());

        }
        else {
            ROS_ERROR("[%s]: can't find 'search_for_color_topic' parameter", ros::this_node::getName().c_str());
            ros::shutdown();
        }
    }

    void run() {
        ros::Rate loopRate(100);
        while(ros::ok()) {
            if(_gotMission) {
                robotican_common::searchForColorResponse response;
                robotican_common::searchForColorRequest request;
                request.color = _color;
                if(_searchForColorClient.call(request, response)) {
                    if(response.valid) {
                        _gotMission = false;
                        move_base_msgs::MoveBaseGoal goal;
                        goal.target_pose.header.frame_id = response.pose.header.frame_id;
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = response.pose.pose;
                        ROS_INFO("[%s]: Sending goal", ros::this_node::getName().c_str());
                        _moveBaseClient.sendGoal(goal);
                        _moveBaseClient.waitForResult();
                        if(_moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                            ROS_INFO("[%s]: SUCCEEDED", ros::this_node::getName().c_str());
                        }
                        else {
                            ROS_ERROR("[%s]: FAILED ", ros::this_node::getName().c_str());
                        }
                    }

                }
            }
            loopRate.sleep();
        }

    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "demo_nav_node");



    return 0;
}