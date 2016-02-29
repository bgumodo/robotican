#ifndef _ROS_ric_robot_ric_status_h
#define _ROS_ric_robot_ric_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

  class ric_status : public ros::Msg
  {
    public:
      uint8_t faults;
      float sensors_battery_voltage;
      float rover_battery_voltage;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->faults >> (8 * 0)) & 0xFF;
      offset += sizeof(this->faults);
      union {
        float real;
        uint32_t base;
      } u_sensors_battery_voltage;
      u_sensors_battery_voltage.real = this->sensors_battery_voltage;
      *(outbuffer + offset + 0) = (u_sensors_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sensors_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sensors_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sensors_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensors_battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_rover_battery_voltage;
      u_rover_battery_voltage.real = this->rover_battery_voltage;
      *(outbuffer + offset + 0) = (u_rover_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rover_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rover_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rover_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rover_battery_voltage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->faults =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->faults);
      union {
        float real;
        uint32_t base;
      } u_sensors_battery_voltage;
      u_sensors_battery_voltage.base = 0;
      u_sensors_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sensors_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sensors_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sensors_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sensors_battery_voltage = u_sensors_battery_voltage.real;
      offset += sizeof(this->sensors_battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_rover_battery_voltage;
      u_rover_battery_voltage.base = 0;
      u_rover_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rover_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rover_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rover_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rover_battery_voltage = u_rover_battery_voltage.real;
      offset += sizeof(this->rover_battery_voltage);
     return offset;
    }

    const char * getType(){ return "ric_robot/ric_status"; };
    const char * getMD5(){ return "67d6e73ce3006094432bbce95b6807f9"; };

  };

}
#endif