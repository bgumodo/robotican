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
#include <string>
#include <stdlib.h>
#include <sstream>
#include <fstream>

#include <cmath>

#include "yaml-cpp/yaml.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

#include <ros/package.h>
#include <XmlRpcValue.h>

#include <dynamixel_pro_controller/dynamixel_pro_controller.h>
#include <dynamixel_pro_driver/dynamixel_pro_driver.h>


using namespace dynamixel_pro_controller;
using namespace std;

DynamixelProController::DynamixelProController(hardware_interface::JointStateInterface *jointStateInterface,
                                               hardware_interface::PosVelJointInterface *posVelJointInterface)
{
    shutting_down = false;
    nh = new ros::NodeHandle("~");
    _time = ros::Time::now();

    _jointStateInterface = jointStateInterface;
    _posVelJointInterface = posVelJointInterface;
    //load the file containing model info, we're not using the param server here
    string path = ros::package::getPath("dynamixel_pro_controller");
    path += "/config/motor_data.yaml";

    YAML::Node doc;

#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::LoadFile(path);
#else
    ifstream fin(path.c_str());
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif

    for (int i = 0; i < doc.size(); i++)
    {
        dynamixel_spec spec;

        // Load the basic specs of this motor type
        doc[i]["name"] >> spec.name;
        doc[i]["model_number"] >> spec.model_number;
        doc[i]["cpr"]  >> spec.cpr;
        doc[i]["gear_reduction"]  >> spec.gear_reduction;

        model_number2specs[spec.model_number] = spec;
    }

    //load all the info from the param server, with defaults
    nh->param<double>("publish_rate", publish_rate, 50.5);
    nh->param<bool>("publish_velocities", publish_velocities, false);

    string device;
    int baudrate;
    int timeout_ms;
    nh->param<std::string>("device", device, "/dev/ttyUSB0");
    nh->param<int>("baudrate", baudrate, 1000000);
    nh->param<int>("serial_timeout_ms", timeout_ms, 1);
    stringstream ss;
    ss << baudrate;

    driver = new dynamixel_pro_driver::DynamixelProDriver(device, ss.str(), timeout_ms);

    int num_motors = 0;

    // read in the information regarding the servos that we're supposed to
    // connect to
    if (nh->hasParam("servos"))
    {
        XmlRpc::XmlRpcValue servos;
        nh->getParam("servos", servos);
        //If there is no servos array in the param server, return
        if (!servos.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Invalid/missing servo information on the param server");
            ROS_BREAK();
        }

        num_motors = servos.size();
        //For every servo, load and verify its information
        for (int i = 0; i < servos.size(); i++)
        {
            dynamixel_info info;

            if (!servos[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("Invalid/Missing info-struct for servo index %d", i);
                ROS_BREAK();
            }

            if (!servos[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
                ROS_ERROR("Invalid/Missing id for servo index %d", i);
                ROS_BREAK();
            }
            else
            {
                //store the servo's ID
                info.id = static_cast<int>(servos[i]["id"]);
            }

            if (!servos[i]["joint_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("Invalid/Missing joint name for servo index %d, id: %d", i, info.id);
                ROS_BREAK();
            }
            else
            {
                //store the servo's corresponding joint
                info.joint_name = static_cast<std::string>(servos[i]["joint_name"]);
            }

            //Ping the servo to make sure that we can actually connect to it
            // and that it is alive and well on our bus
            if (driver->ping(info.id))
            {
                bool success = true;
                success &= driver->getModelNumber(info.id, info.model_number);
                success &= driver->getModelInfo(info.id, info.model_info);
                success &= model_number2specs.find(info.model_number) != model_number2specs.end();
                //make sure that we are successfully able to pull all the model
                //information

                if (success)
                {
                    //set up the lookup tables that we'll use later in the code
                    //to look up how to operate each joint
                    info.cpr = model_number2specs[info.model_number].cpr;
                    info.gear_reduction = model_number2specs[info.model_number].gear_reduction;

                    joint2dynamixel[info.joint_name] = info;

                    //initialize the status struct with the starting/ default
                    //status
                    dynamixel_status status;
                    status.id = info.id;
                    status.mode = UNKOWN;
                    status.torque_enabled = false;

                    id2status[info.id] = status;
                }
                else
                {
                    ROS_ERROR("Failed to load model information for dynamixel id %d", info.id);
                    ROS_ERROR("Model Number: %d, Model Info: %d ", info.model_number, info.model_info);
                    if (model_number2specs.find(info.model_number) != model_number2specs.end())
                        ROS_ERROR("Info is in database");
                    else
                        ROS_ERROR("Info is not in database");

                }
            }
            else
                ROS_ERROR("Cannot ping dyanmixel id: %d", info.id);
        }
    }
    else
    {
        ROS_ERROR("No servos details loaded to param server");
        ROS_BREAK();
    }

    for(std::map<std::string, dynamixel_info>::iterator it=joint2dynamixel.begin(); it != joint2dynamixel.end(); ++it) {
        string jointName = it->first;
        _jointsInfo.insert(std::pair<string, JointInfo_t>(jointName, JointInfo_t()));
        hardware_interface::JointStateHandle jointStateHandle(jointName, &_jointsInfo[jointName].position, &_jointsInfo[jointName].velocity,&_jointsInfo[jointName].effort);
        _jointStateInterface->registerHandle(jointStateHandle);
        hardware_interface::PosVelJointHandle jointHandle(_jointStateInterface->getHandle(jointName), &_jointsInfo[jointName].cmd_pos, &_jointsInfo[jointName].cmd_vel);
        _posVelJointInterface->registerHandle(jointHandle);

    }
    registerInterface(_jointStateInterface);
    registerInterface(_posVelJointInterface);


    //advertise the sensor feedback topic
//    jointStatePublisher  = nh->advertise<sensor_msgs::JointState>("/joint_states", 1);

    //Start listening to command messages. There is a queue size of 1k so that
    //we don't accidentally miss commands that are sent to us in batches for
    //many joints at once.
//    jointStateSubscriber = nh->subscribe<sensor_msgs::JointState>("/joint_commands",
//        1000, &DynamixelProController::jointStateCallback, this);
    chainEnableSubscriber = nh->subscribe<ChainEnable>("joint_enable",
        1000, &DynamixelProController::chainEnableCallback, this);
    chainLimitsSubscriber = nh->subscribe<ChainLimits>("joint_limits",
        1000, &DynamixelProController::chainLimitCallback, this);
}

DynamixelProController::~DynamixelProController()
{
    shutting_down = false;
    delete nh;

    ros::Duration(0.1).sleep();//just in case. This should be changed to
    //something a tad more deterministic than this
    for (map<string, dynamixel_info>::iterator iter = joint2dynamixel.begin(); iter != joint2dynamixel.end(); iter++)
    {
        driver->setTorqueEnabled(iter->second.id, 0);
    }
    delete driver;
}

void DynamixelProController::startBroadcastingJointStates()
{
    broadcastTimer = nh->createTimer(ros::Duration(1.0 / publish_rate), &DynamixelProController::publishJointStates, this);
    broadcastTimer.start();
}

void DynamixelProController::jointStateCallback(sensor_msgs::JointState &msg)
{
    if (shutting_down)
        return;

    bool has_pos = false, has_vel = false, has_torque = false;
    control_mode new_mode = UNKOWN;

    //figure out which value is going to be our setpoint
    if (msg.position.size() > 0)
        has_pos = true;
    if (msg.velocity.size() > 0)
        has_vel = true;
    else if (msg.effort.size() > 0)
        has_torque = true;

    //figure out which mode we are going to operate the servos in
    if (has_pos)
        new_mode = POSITION_CONTROL;
    else if (has_vel)
        new_mode = VELOCITY_CONTROL;
    else if (has_torque)
        new_mode = TORQUE_CONTROL;

    vector<int> ids, velocities, positions, torques;

    //actually send the commands to the joints
    for (int i = 0; i < msg.name.size(); i++)
    {
        //lookup the information for that particular joint to be able to control it
        string name = msg.name[i];
        dynamixel_info info = joint2dynamixel[name];
        dynamixel_status &status = id2status[info.id];

        //change to new mode if needed
        if(status.mode != new_mode && new_mode != UNKOWN)
        {
            if (status.torque_enabled)//you can't seem to change modes while the the servo is enabled
                driver->setTorqueEnabled(info.id, 0);
            status.torque_enabled = false;

            driver->setOperatingMode(info.id, new_mode);//the enum is set up to correspond to the operating modes
            status.mode = new_mode;
        }

        //enable torque if needed
        if (!status.torque_enabled)
            driver->setTorqueEnabled(info.id, 1);
        status.torque_enabled = true;

        //prepare data to be sent to the motor
        ids.push_back(info.id);

        if (has_pos)
        {
            double rad_pos = msg.position[i];
            int32_t pos = posToTicks(rad_pos, info);
            positions.push_back(pos);
        }
        if (has_vel)
        {

            double rad_s_vel = msg.velocity[i];
            int vel = static_cast<int>(rad_s_vel / 2.0 / M_PI * 60.0 * info.gear_reduction);
            velocities.push_back(vel);
        }
        if (has_torque)
        {
            //replace the below with proper code when you can figure out the units
            static bool first_run = true;
            if (first_run)
                ROS_WARN("Dynamixel pro controller torque control mode not implemented");
        }
    }

    //send the setpoints in monolithic packets to reduce bandwidth
    if (has_pos && has_vel)
    {
        //send both a position and a velocity limit
        vector< vector<int> > data;
        for (int i = 0; i < ids.size(); i++)
        {
            if(velocities[i] > 0) {
                vector<int> temp;
                temp.push_back(ids[i]);//order matters here
                temp.push_back(positions[i]);
                temp.push_back(abs(velocities[i])); //velocity limits should always be positive
                data.push_back(temp);
            }
        }
        if(data.size() > 0 ) {
            driver->setMultiPositionVelocity(data);
        }
        else return;
    }
    else
    {
        //send only a position setpoint
        if (has_pos)
        {
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(positions[i]);
                data.push_back(temp);
            }
            driver->setMultiPosition(data);
        }

        //send only a velocity setpoint
        if (has_vel)
        {
            vector< vector<int> > data;
            for (int i = 0; i < ids.size(); i++)
            {
                vector<int> temp;
                temp.push_back(ids[i]);
                temp.push_back(velocities[i]);
                data.push_back(temp);
            }
            driver->setMultiVelocity(data);
        }
    }

    if (has_torque)
    {
        //TODO add this functionality
    }
}

void DynamixelProController::chainEnableCallback(const ChainEnable::ConstPtr& msg)
{
    std::vector<std::vector<int> > enables;
    enables.reserve(msg->list.size());
    for(std::vector<JointEnable>::const_iterator ii = msg->list.begin(); ii != msg->list.end(); ++ii)
    {
        const std::string& name = ii->name;
        const dynamixel_info &info = joint2dynamixel[name];
        dynamixel_status &status = id2status[info.id];

        std::vector<int> command(2, 0);
        command[0] = info.id;
        command[1] = static_cast<int>(ii->enable);
        enables.push_back(command);
    }

    if(driver->setMultiTorqueEnabled(enables))
    {
        for(std::vector<JointEnable>::const_iterator ii = msg->list.begin(); ii != msg->list.end(); ++ii)
        {
            const std::string& name = ii->name;
            const dynamixel_info &info = joint2dynamixel[name];
            dynamixel_status &status = id2status[info.id];
            status.torque_enabled = ii->enable;
        }
    }
}

void DynamixelProController::chainLimitCallback(const ChainLimits::ConstPtr& msg)
{
    for(std::vector<JointLimits>::const_iterator ii = msg->list.begin(); ii != msg->list.end(); ++ii)
    {
        const std::string& name = ii->name;
        const dynamixel_info &info = joint2dynamixel[name];
        dynamixel_status &status = id2status[info.id];

        int32_t min_limit = posToTicks(ii->min_angle, info);
        int32_t max_limit = posToTicks(ii->max_angle, info);

        driver->setAngleLimits(info.id, min_limit, max_limit);
    }
}

ros::Time DynamixelProController::getTime() {
    return ros::Time::now();
}

ros::Duration DynamixelProController::getPeriod() {
    ros::Time now = ros::Time::now();
    ros::Duration period = now - _time;
    _time = now;
    return period;
}


void DynamixelProController::read() {
    //don't access the driver after its been cleaned up
    if (shutting_down)
        return;

    //sensor_msgs::JointState msg;
    //msg.header.stamp = ros::Time::now();

    //Iterate over all connected servos
    for (map<string, dynamixel_info>::iterator iter = joint2dynamixel.begin(); iter != joint2dynamixel.end(); iter++)
    {
        string joint_name = iter->first;
        dynamixel_info info = iter->second;

        //get the position and conditionally the velocity and then publish them
        //under the joint name which we just looked up
        int32_t position, velocity;
        if (driver->getPosition(info.id, position))
        {
            double rad_pos = posToRads(position, info);
            _jointsInfo[joint_name].position = rad_pos;
            //msg.name.push_back(joint_name);
            //msg.position.push_back(rad_pos);
            if (publish_velocities && driver->getVelocity(info.id, velocity))
            {

                double rad_vel = ((double) velocity) * 2.0 * M_PI / 60.0 / info.gear_reduction;
                _jointsInfo[joint_name].velocity = rad_vel;
                //msg.velocity.push_back(rad_vel);
            }
        }
    }
    //jointStatePublisher.publish(msg);
}

void DynamixelProController::write() {
    sensor_msgs::JointState msg;
    for(std::map<std::string, JointInfo_t>::iterator it = _jointsInfo.begin(); it != _jointsInfo.end(); ++it) {
        msg.name.push_back(it->first);
        msg.position.push_back(it->second.cmd_pos);
        if(it->second.cmd_vel > 0) {
			msg.velocity.push_back(it->second.cmd_vel);
	    }
	    else {
			msg.velocity.push_back(0.2);
		}
    }
    
    //sensor_msgs::JointState::ConstPtr send(&msg);
    jointStateCallback(msg);
}

void DynamixelProController::publishJointStates(const ros::TimerEvent& e)
{
//    //don't access the driver after its been cleaned up
//    if (shutting_down)
//        return;
//
//    //sensor_msgs::JointState msg;
//    //msg.header.stamp = ros::Time::now();
//
//    //Iterate over all connected servos
//    for (map<string, dynamixel_info>::iterator iter = joint2dynamixel.begin(); iter != joint2dynamixel.end(); iter++)
//    {
//        string joint_name = iter->first;
//        dynamixel_info info = iter->second;
//
//        //get the position and conditionally the velocity and then publish them
//        //under the joint name which we just looked up
//        int32_t position, velocity;
//        if (driver->getPosition(info.id, position))
//        {
//            double rad_pos = posToRads(position, info);
//            _jointsInfo[joint_name].position = rad_pos;
//            //msg.name.push_back(joint_name);
//            //msg.position.push_back(rad_pos);
//            if (publish_velocities && driver->getVelocity(info.id, velocity))
//            {
//
//                double rad_vel = ((double) velocity) * 2.0 * M_PI / 60.0 / info.gear_reduction;
//                _jointsInfo[joint_name].velocity = rad_vel;
//                //msg.velocity.push_back(rad_vel);
//            }
//        }
//    }
//    //jointStatePublisher.publish(msg);
}

/**
 * Converts a position angle in radians to ticks for a given motor type.
 */
int32_t DynamixelProController::posToTicks(double rads, const dynamixel_info& info) const
{
    const double ToTicks = info.cpr / 2.0;
    return static_cast<int>(round((rads / M_PI) * ToTicks));
}

/**
 * Converts a position angle in ticks to radians for a given motor type.
 */
double DynamixelProController::posToRads(int32_t ticks, const dynamixel_info& info) const
{
    const double FromTicks = 1.0 / (info.cpr / 2.0);
    return static_cast<double>(ticks) * FromTicks * M_PI;
}

//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "dynamixel_pro_controller");
//    DynamixelProController controller;
//    controller.startBroadcastingJointStates();
//    controller_manager::ControllerManager controllerManager(&controller);
//    ros::AsyncSpinner spinner(2);
//    spinner.start();
//    ros::Rate rate(50);
//    while(ros::ok()) {
//        controller.read();
//        controllerManager.update(controller.getTime(), controller.getPeriod());
//        controller.write();
//        rate.sleep();
//    }
//
    //ros::spin(); //use a single threaded spinner as I'm pretty sure this code isn't thread safe.
//}
