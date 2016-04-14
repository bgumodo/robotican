//
// Created by tom on 14/04/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H
#define ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H

#include <ric_board/Motor.h>
#include <robotican_hardware_interface/robot_base.h>
#include <hardware_interface/joint_command_interface.h>

namespace robotican_hardware {
    class LiziRobot : public RobotBase {
    private:
        std::pair<std::string, JointInfo_t> _rearLeftMotorJointInfo;
        std::pair<std::string, JointInfo_t> _rearRightMotorJointInfo;
        ros::Publisher _rearLeftMotorCmd;

        ros::Publisher _rearRightMotorCmd;
        ros::Subscriber _rearLeftMotorState;
        ros::Subscriber _rearRightMotorState;
        void rearRightMotorStateCallback(const ric_board::Motor::ConstPtr &msg);

        void rearLeftMotorStateCallback(const ric_board::Motor::ConstPtr &msg);

    protected:
        hardware_interface::PositionJointInterface _positionJointInterface;

    public:
        LiziRobot();
        virtual ~LiziRobot();
        virtual void registerInterfaces();
        virtual void read();
        virtual void write();
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_LIZIROBOT_H
