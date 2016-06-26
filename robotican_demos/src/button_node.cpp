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
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0168, 0.093));

    tf::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(quaternion);
    transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_finger_link", "left_finger_tp"));

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
    transformBroadcasterTimer.start();

    ros::ServiceClient serviceClient = nodeHandle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    ROS_INFO("[%s]: Waiting for gazebo/delete_mode service....", ros::this_node::getName().c_str());
    serviceClient.waitForExistence();

    ros::Rate loopRate(100);
    float button_x, button_y, button_z, buttonEpsilon;

    ros::param::param<float>("button_x", button_x, 0.0);
    ros::param::param<float>("button_y", button_y, 0.0);
    ros::param::param<float>("button_z", button_z, 0.0);
    ros::param::param<float>("button_epsilon", buttonEpsilon, 0.1);
    bool spawnObject = false;
    bool inPose = false;
    ROS_INFO("[%s]: Robotican node button is active", ros::this_node::getName().c_str());



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
                    if (!spawnObject) {
                        char exec[128] = {'\0'};

                        sprintf(exec, "rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can2 -x %f -y %f -z %f",
                                button_x, button_y, button_z);

                        FILE *process = popen(exec, "r");
                        if(process == 0) {
                            ROS_ERROR("[%s]: can't start the procces shuting down the node :(", ros::this_node::getName().c_str());
                            ros::shutdown();
                        }
                        else spawnObject = true;

                    }
                    else {
                        gazebo_msgs::DeleteModelRequest request;
                        request.model_name = "coke_can2";
                        gazebo_msgs::DeleteModelResponse response;

                        if(serviceClient.call(request, response)) {
                            if(response.success) {
                                spawnObject = false;
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