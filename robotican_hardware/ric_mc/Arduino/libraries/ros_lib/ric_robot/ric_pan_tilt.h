#ifndef _ROS_ric_robot_ric_pan_tilt_h
#define _ROS_ric_robot_ric_pan_tilt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

  class ric_pan_tilt : public ros::Msg
  {
    public:
      float pan_angle;
      float tilt_angle;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan_angle;
      u_pan_angle.real = this->pan_angle;
      *(outbuffer + offset + 0) = (u_pan_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pan_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pan_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pan_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pan_angle);
      union {
        float real;
        uint32_t base;
      } u_tilt_angle;
      u_tilt_angle.real = this->tilt_angle;
      *(outbuffer + offset + 0) = (u_tilt_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tilt_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tilt_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tilt_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tilt_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan_angle;
      u_pan_angle.base = 0;
      u_pan_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pan_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pan_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pan_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pan_angle = u_pan_angle.real;
      offset += sizeof(this->pan_angle);
      union {
        float real;
        uint32_t base;
      } u_tilt_angle;
      u_tilt_angle.base = 0;
      u_tilt_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tilt_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tilt_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tilt_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tilt_angle = u_tilt_angle.real;
      offset += sizeof(this->tilt_angle);
     return offset;
    }

    const char * getType(){ return "ric_robot/ric_pan_tilt"; };
    const char * getMD5(){ return "f191de4d1f51ebd5f8f3b12305019bf6"; };

  };

}
#endif