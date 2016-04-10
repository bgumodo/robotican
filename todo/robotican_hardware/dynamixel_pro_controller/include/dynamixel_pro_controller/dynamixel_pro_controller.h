/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef DYNAMIXEL_PRO_CONTROLLER_H_
#define DYNAMIXEL_PRO_CONTROLLER_H_

#include <string>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_pro_controller/ChainEnable.h>
#include <dynamixel_pro_controller/ChainLimits.h>
#include <dynamixel_pro_driver/dynamixel_pro_driver.h>

#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>


namespace dynamixel_pro_controller
{

    struct JointInfo_t {
        double position;
        double effort;
        double velocity;
        double cmd_pos;
        double cmd_vel;
        JointInfo_t() {
            position = effort = velocity = cmd_pos = 0;
	    cmd_vel = 0;
        }
    };

/**
 * This class is a node that provides a ros interface to the dynamixel
 * Pro driver.
 */
class DynamixelProController : public hardware_interface::RobotHW
{
public:
    DynamixelProController(hardware_interface::JointStateInterface *jointStateInterface,
                           hardware_interface::PosVelJointInterface *posVelJointInterface);
    ~DynamixelProController();
    ros::Time getTime();
    ros::Duration getPeriod();

    /**
     * Start broadcasting JointState messages corresponding to the
     * connected dynamixels
     */
    void startBroadcastingJointStates();
    void read();
    void write();
private:
    /**
     * callback for recieving a command
     */
    void jointStateCallback(sensor_msgs::JointState &msg);
    void chainEnableCallback(const dynamixel_pro_controller::ChainEnable::ConstPtr& msg);
    void chainLimitCallback(const dynamixel_pro_controller::ChainLimits::ConstPtr& msg);

    /**
     * TimeEvent callback for publishing a joint state.
     */
    void publishJointStates(const ros::TimerEvent& e);

    /**
     * Struct that describes the dynamixel motor's static and physical
     * properties
     */
    struct dynamixel_spec
    {
        std::string name;
        uint16_t model_number;
        int cpr;
        double gear_reduction;
    };

    /**
     * Struct that describes each servo's place in the system including
     * which joint it corresponds to.
     */
    struct dynamixel_info
    {
        int id;
        std::string joint_name;
        std::string model_name;
        uint16_t model_number;
        uint32_t model_info;
        int cpr;
        double gear_reduction;
    };

    /**
     * The different control modes available on the dynamixel servos.
     */
    enum control_mode
    {
        POSITION_CONTROL = 3,
        VELOCITY_CONTROL = 1,
        TORQUE_CONTROL = 0,
        UNKOWN  = -1
    };

    /**
     * A struct that describes the state of a dynamixel servo motor
     */
    struct dynamixel_status
    {
        int id;
        control_mode mode;
        bool torque_enabled;
    };

    int32_t posToTicks(double rads, const dynamixel_info& info) const;
    double posToRads(int32_t ticks, const dynamixel_info& info) const;

    ros::NodeHandle *nh;
    dynamixel_pro_driver::DynamixelProDriver *driver;
    double publish_rate;
    bool publish_velocities;

    volatile bool shutting_down;

    std::map<std::string, dynamixel_info> joint2dynamixel;
    std::map<uint16_t, dynamixel_spec> model_number2specs;
    std::map<int, dynamixel_status> id2status;

    ros::Publisher jointStatePublisher;
    ros::Subscriber jointStateSubscriber;
    ros::Subscriber chainEnableSubscriber;
    ros::Subscriber chainLimitsSubscriber;
    ros::Timer broadcastTimer;
    ros::Time _time;
    std::map<std::string, JointInfo_t> _jointsInfo;
    hardware_interface::JointStateInterface* _jointStateInterface;
    hardware_interface::PosVelJointInterface* _posVelJointInterface;


};

}

#endif //DYNAMIXEL_PRO_CONTROLLER_H_
