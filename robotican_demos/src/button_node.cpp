//
// Created by tom on 26/06/16.
//

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/DeleteModel.h>
#include <tf/transform_broadcaster.h>

#define BUTTON_EP 0.1

void onTfBroadcaster(const ros::TimerEvent& event) {
    static tf::TransformBroadcaster transformBroadcaster;
    tf::Transform transformFinger;
    transformFinger.setOrigin(tf::Vector3(0.0, 0.0168, 0.093));

    tf::Quaternion quaternionFinger;
    quaternionFinger.setRPY(0.0, 0.0, 0.0);
    transformFinger.setRotation(quaternionFinger);
    transformBroadcaster.sendTransform(tf::StampedTransform(transformFinger, ros::Time::now(), "left_finger_link", "left_finger_tp"));

    double button_x, button_y, button_z;

    ros::param::param<double>("button_x", button_x, -4.9145);
    ros::param::param<double>("button_y", button_y, 8.2331);
    ros::param::param<double>("button_z", button_z, 0.7251);

    tf::Transform transformButton;
    tf::Quaternion quaternionButton;

    transformButton.setOrigin(tf::Vector3(button_x, button_y, button_z));
    quaternionButton.setRPY(0.0, 0.0, 0.0);
    transformButton.setRotation(quaternionButton);
    transformBroadcaster.sendTransform(tf::StampedTransform(transformButton, ros::Time::now(), "map", "button"));
}


float distance(float xDes, float yDes, float zDes, float xCur, float yCur, float zCur) {
    return sqrtf(powf(xDes - xCur, 2) + powf(yDes - yCur, 2) + powf(zDes - zCur, 2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "button_node");
    ros::NodeHandle nodeHandle;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Timer transformBroadcasterTimer = nodeHandle.createTimer(ros::Duration(0.1), onTfBroadcaster);


    ros::ServiceClient serviceClient = nodeHandle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    ROS_INFO("[%s]: Waiting for gazebo/delete_mode service....", ros::this_node::getName().c_str());
    serviceClient.waitForExistence();

    ros::Rate loopRate(100);
    double button_x, button_y, button_z, buttonEpsilon;

    ros::param::param<double>("button_x", button_x, -4.9145);
    ros::param::param<double>("button_y", button_y, 8.2331);
    ros::param::param<double>("button_z", button_z, 0.7251);
    ros::param::param<double>("button_epsilon", buttonEpsilon, 0.1);
    bool deleteObject = false;
    bool inPose = false;
    ROS_INFO("[%s]: Robotican node button is active", ros::this_node::getName().c_str());


    transformBroadcasterTimer.start();
    while (ros::ok()) {
        try {
            listener.lookupTransform("/map", "left_finger_tp", ros::Time(0), transform);
            float radToButton = distance(button_x, button_y, button_z, (float) transform.getOrigin().x(),
                                         (float) transform.getOrigin().y(), (float) transform.getOrigin().z());

#ifdef DEBUG_NODE_BUTTON
            std::string nodeName = ros::this_node::getName();
            ROS_INFO("[%s]: current transform { x: %.4f, y: %.4f, z: %.4f }", nodeName.c_str(),
                     transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            ROS_INFO("[%s]: distance is %.4f", nodeName.c_str(), radToButton);
#endif


            if (radToButton <= buttonEpsilon) {
                if (!inPose) {
                    inPose = true;
                    if (!deleteObject) {
                        gazebo_msgs::DeleteModelRequest request;
                        request.model_name = "unit_box_2";
                        gazebo_msgs::DeleteModelResponse response;

                        if(serviceClient.call(request, response)) {
                            if(response.success) {
                                deleteObject = true;
                            }
                            ROS_INFO("[%s]: %s", ros::this_node::getName().c_str(), response.status_message.c_str());
                        }
                        else {
                            ROS_ERROR("[%s]: Can't call service gazebo/delete_mode shuting down the node :(", ros::this_node::getName().c_str());
                            ros::shutdown();
                        }

                    }
                }
            }
            else {
                inPose = false;
            }
        }
        catch (tf::TransformException ex) {

        }


        loopRate.sleep();
    }
}