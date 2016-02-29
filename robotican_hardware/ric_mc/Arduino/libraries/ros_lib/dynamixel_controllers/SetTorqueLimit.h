#ifndef _ROS_SERVICE_SetTorqueLimit_h
#define _ROS_SERVICE_SetTorqueLimit_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETTORQUELIMIT[] = "dynamixel_controllers/SetTorqueLimit";

  class SetTorqueLimitRequest : public ros::Msg
  {
    public:
      float torque_limit;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_torque_limit = (int32_t *) &(this->torque_limit);
      int32_t exp_torque_limit = (((*val_torque_limit)>>23)&255);
      if(exp_torque_limit != 0)
        exp_torque_limit += 1023-127;
      int32_t sig_torque_limit = *val_torque_limit;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_torque_limit<<5) & 0xff;
      *(outbuffer + offset++) = (sig_torque_limit>>3) & 0xff;
      *(outbuffer + offset++) = (sig_torque_limit>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_torque_limit<<4) & 0xF0) | ((sig_torque_limit>>19)&0x0F);
      *(outbuffer + offset++) = (exp_torque_limit>>4) & 0x7F;
      if(this->torque_limit < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_torque_limit = (uint32_t*) &(this->torque_limit);
      offset += 3;
      *val_torque_limit = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_torque_limit |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_torque_limit |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_torque_limit |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_torque_limit = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_torque_limit |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_torque_limit !=0)
        *val_torque_limit |= ((exp_torque_limit)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->torque_limit = -this->torque_limit;
     return offset;
    }

    const char * getType(){ return SETTORQUELIMIT; };
    const char * getMD5(){ return "7ac67170532bb79d95db2a425915bbd6"; };

  };

  class SetTorqueLimitResponse : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETTORQUELIMIT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTorqueLimit {
    public:
    typedef SetTorqueLimitRequest Request;
    typedef SetTorqueLimitResponse Response;
  };

}
#endif
