//
// Created by tom on 13/06/16.
//


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>


typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> Client;

class ObjectRecognition {
private:
    ros::NodeHandle _nodeHandle;
    Client _client;

    void doneCallBack(const actionlib::SimpleClientGoalState &state, const object_recognition_msgs::ObjectRecognitionResult::ConstPtr result) {
        ROS_INFO("[%s]: State is: %s", ros::this_node::getName().c_str(), state.getText().c_str());
        size_t objectSize = result->recognized_objects.objects.size();

        ROS_INFO("[%s]: recognize object number: %ld", ros::this_node::getName().c_str(), objectSize);

        for(int i = 0; i < objectSize; ++i) {
            object_recognition_msgs::RecognizedObject object = result->recognized_objects.objects[i];
            ROS_INFO("[%s]: object {x: %f }", ros::this_node::getName().c_str(), object.pose.pose.pose.position.x);
        }
    }

public:

    ObjectRecognition() : _client("recognize_objects", true) {
        _client.waitForServer();
    }

    void sendGoal() {
        object_recognition_msgs::ObjectRecognitionGoal goal;
        goal.use_roi = false;
        _client.sendGoal(goal, boost::bind(&ObjectRecognition::doneCallBack, this, _1, _2), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
        _client.waitForResult();
    }

    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_recognition_node");
    ObjectRecognition objectRecognition;
    objectRecognition.sendGoal();
    objectRecognition.run();
    return 0;
}

