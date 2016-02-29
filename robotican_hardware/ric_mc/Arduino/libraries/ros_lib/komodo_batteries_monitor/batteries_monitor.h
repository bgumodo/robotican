#ifndef _ROS_komodo_batteries_monitor_batteries_monitor_h
#define _ROS_komodo_batteries_monitor_batteries_monitor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace komodo_batteries_monitor
{

  class batteries_monitor : public ros::Msg
  {
    public:
      ros::Time pc_bat_time;
      float pc_bat_v;
      ros::Time arm_bat_time;
      float arm_bat_v;
      ros::Time rover_bat_time;
      float rover_bat_v;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pc_bat_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pc_bat_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pc_bat_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pc_bat_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pc_bat_time.sec);
      *(outbuffer + offset + 0) = (this->pc_bat_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pc_bat_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pc_bat_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pc_bat_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pc_bat_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_pc_bat_v;
      u_pc_bat_v.real = this->pc_bat_v;
      *(outbuffer + offset + 0) = (u_pc_bat_v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pc_bat_v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pc_bat_v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pc_bat_v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pc_bat_v);
      *(outbuffer + offset + 0) = (this->arm_bat_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arm_bat_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arm_bat_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arm_bat_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arm_bat_time.sec);
      *(outbuffer + offset + 0) = (this->arm_bat_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arm_bat_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arm_bat_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arm_bat_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arm_bat_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_arm_bat_v;
      u_arm_bat_v.real = this->arm_bat_v;
      *(outbuffer + offset + 0) = (u_arm_bat_v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm_bat_v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arm_bat_v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arm_bat_v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arm_bat_v);
      *(outbuffer + offset + 0) = (this->rover_bat_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rover_bat_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rover_bat_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rover_bat_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rover_bat_time.sec);
      *(outbuffer + offset + 0) = (this->rover_bat_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rover_bat_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rover_bat_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rover_bat_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rover_bat_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_rover_bat_v;
      u_rover_bat_v.real = this->rover_bat_v;
      *(outbuffer + offset + 0) = (u_rover_bat_v.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rover_bat_v.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rover_bat_v.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rover_bat_v.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rover_bat_v);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->pc_bat_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->pc_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pc_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pc_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pc_bat_time.sec);
      this->pc_bat_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->pc_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pc_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pc_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pc_bat_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_pc_bat_v;
      u_pc_bat_v.base = 0;
      u_pc_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pc_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pc_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pc_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pc_bat_v = u_pc_bat_v.real;
      offset += sizeof(this->pc_bat_v);
      this->arm_bat_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->arm_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->arm_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->arm_bat_time.sec);
      this->arm_bat_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->arm_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->arm_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->arm_bat_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_arm_bat_v;
      u_arm_bat_v.base = 0;
      u_arm_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arm_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arm_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->arm_bat_v = u_arm_bat_v.real;
      offset += sizeof(this->arm_bat_v);
      this->rover_bat_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->rover_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rover_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->rover_bat_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->rover_bat_time.sec);
      this->rover_bat_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->rover_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rover_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->rover_bat_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->rover_bat_time.nsec);
      union {
        float real;
        uint32_t base;
      } u_rover_bat_v;
      u_rover_bat_v.base = 0;
      u_rover_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rover_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rover_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rover_bat_v.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rover_bat_v = u_rover_bat_v.real;
      offset += sizeof(this->rover_bat_v);
     return offset;
    }

    const char * getType(){ return "komodo_batteries_monitor/batteries_monitor"; };
    const char * getMD5(){ return "ec3967ff434930a33d16a16a1d3824b6"; };

  };

}
#endif