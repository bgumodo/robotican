#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavNode {
private:
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _navSub;
    MoveBaseClient _ac;
    ros::AsyncSpinner _spinner;
    bool _isFinish;

    void onNav(const geometry_msgs::PoseStamped::ConstPtr &pos) {
      if(_isFinish) {
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose = *pos;
          _ac.sendGoal(goal);
          _isFinish = false;
          _ac.waitForResult();

          if(_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
              ROS_INFO("[%s]: SUCCEEDED", ros::this_node::getName().c_str());

          }
          else {
              ROS_INFO("[%s]: ABORTED", ros::this_node::getName().c_str());
          }

          _isFinish = true;
      }
      else  {
          ROS_ERROR("[%s]: Nav not finish with the previous mission", ros::this_node::getName().c_str());
      }
    }

public:
    NavNode() : _spinner(1), _ac("move_base", true) {
        _isFinish = true;
        _spinner.start();
        std::string topicName;
        ros::param::param<std::string>("nav_topic_name", topicName, "nav_command");
        _navSub = _nodeHandle.subscribe(topicName, 10, &NavNode::onNav, this);
        _ac.waitForServer();

    }

    void run() {
      ros::spin();
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  NavNode navNode;
  navNode.run();

  return 0;
}