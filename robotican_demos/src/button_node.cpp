//
// Created by tom on 26/06/16.
//

#include <ros/ros.h>
#include <tf/transform_listener.h>

#define BUTTON_EP 0.1

int main(int argc, char **argv) {
    ros::init(argc, argv, "button_node");

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate loopRate(100);

    float button_x, button_y, button_z;

    ros::param::param<float>("button_x", button_x, 0.0);
    ros::param::param<float>("button_y", button_y, 0.0);
    ros::param::param<float>("button_z", button_z, 0.0);
    bool spawnObject = false;
    bool inPose = false;

    while (ros::ok()) {
        try {
            listener.lookupTransform("/map", "left_finger_tp", ros::Time(0), transform);

            if (fabsf((float) (transform.getOrigin().x() - button_x)) <= BUTTON_EP
                && fabsf((float) (transform.getOrigin().y() - button_y)) <= BUTTON_EP
                && fabsf((float) (transform.getOrigin().z() - button_z)) <= BUTTON_EP) {
                if (!inPose) {
                    inPose = true;
                    if (!spawnObject) {
                        spawnObject = true;
                        char exec[128] = {'\0'};

                        sprintf(exec, "rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can2 -x %f -y %f -z %f",
                                button_x, button_y, button_z);

                        FILE *process = popen(exec, "r");
                        if( process == 0) {
                            ROS_ERROR("[%s]: can't start the procces", ros::this_node::getName().c_str());
                        }

                    }
                    else {
                        spawnObject = false;

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