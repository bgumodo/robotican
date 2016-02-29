#ifndef _ROS_SERVICE_SetSpeed_h
#define _ROS_SERVICE_SetSpeed_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETSPEED[] = "dynamixel_controllers/SetSpeed";

  class SetSpeedRequest : public ros::Msg
  {
    public:
      float speed;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_speed = (int32_t *) &(this->speed);
      int32_t exp_speed = (((*val_speed)>>23)&255);
      if(exp_speed != 0)
        exp_speed += 1023-127;
      int32_t sig_speed = *val_speed;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_speed<<5) & 0xff;
      *(outbuffer + offset++) = (sig_speed>>3) & 0xff;
      *(outbuffer + offset++) = (sig_speed>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_speed<<4) & 0xF0) | ((sig_speed>>19)&0x0F);
      *(outbuffer + offset++) = (exp_speed>>4) & 0x7F;
      if(this->speed < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_speed = (uint32_t*) &(this->speed);
      offset += 3;
      *val_speed = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_speed |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_speed = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_speed |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_speed !=0)
        *val_speed |= ((exp_speed)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->speed = -this->speed;
     return offset;
    }

    const char * getType(){ return SETSPEED; };
    const char * getMD5(){ return "4641bb0523a3557209606d9bd91ce33a"; };

  };

  class SetSpeedResponse : public ros::Msg
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

    const char * getType(){ return SETSPEED; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetSpeed {
    public:
    typedef SetSpeedRequest Request;
    typedef SetSpeedResponse Response;
  };

}
#endif
