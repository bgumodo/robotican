#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

int main(int argc, char** argv) {
    ros::init(argc,argv, "check_imu_node");
    ros::NodeHandle handle;
    ros::Subscriber sub = handle.subscribe<sensor_msgs::Imu>("imu_AGQ", 10, imuCallback);
    ros::spin();
    return 0;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    roll = pitch = yaw = 0;
    m.getRPY(roll, pitch, yaw);
    char buff[1024] = {'\0'};
    sprintf(buff, "Roll: %.4f  Pitch: %.4f  Yaw: %.4f", roll * 180 / M_PI ,pitch * 180 / M_PI,yaw * 180 / M_PI);
    ROS_INFO("%s",buff);
}
