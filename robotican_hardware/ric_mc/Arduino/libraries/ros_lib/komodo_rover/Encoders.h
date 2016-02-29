#ifndef _ROS_komodo_rover_Encoders_h
#define _ROS_komodo_rover_Encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace komodo_rover
{

  class Encoders : public ros::Msg
  {
    public:
      float time_delta;
      int32_t left_wheel;
      int32_t right_wheel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_time_delta;
      u_time_delta.real = this->time_delta;
      *(outbuffer + offset + 0) = (u_time_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_left_wheel;
      u_left_wheel.real = this->left_wheel;
      *(outbuffer + offset + 0) = (u_left_wheel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_wheel);
      union {
        int32_t real;
        uint32_t base;
      } u_right_wheel;
      u_right_wheel.real = this->right_wheel;
      *(outbuffer + offset + 0) = (u_right_wheel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wheel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_time_delta;
      u_time_delta.base = 0;
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time_delta = u_time_delta.real;
      offset += sizeof(this->time_delta);
      union {
        int32_t real;
        uint32_t base;
      } u_left_wheel;
      u_left_wheel.base = 0;
      u_left_wheel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_wheel = u_left_wheel.real;
      offset += sizeof(this->left_wheel);
      union {
        int32_t real;
        uint32_t base;
      } u_right_wheel;
      u_right_wheel.base = 0;
      u_right_wheel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wheel = u_right_wheel.real;
      offset += sizeof(this->right_wheel);
     return offset;
    }

    const char * getType(){ return "komodo_rover/Encoders"; };
    const char * getMD5(){ return "265d820a2b35c4bff51c4a1d293ad5c0"; };

  };

}
#endif