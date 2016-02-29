#ifndef _ROS_SERVICE_ric_calib_h
#define _ROS_SERVICE_ric_calib_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

static const char RIC_CALIB[] = "ric_robot/ric_calib";

  class ric_calibRequest : public ros::Msg
  {
    public:
      int16_t com;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_com;
      u_com.real = this->com;
      *(outbuffer + offset + 0) = (u_com.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_com.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->com);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_com;
      u_com.base = 0;
      u_com.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_com.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->com = u_com.real;
      offset += sizeof(this->com);
     return offset;
    }

    const char * getType(){ return RIC_CALIB; };
    const char * getMD5(){ return "0dce049a176069675a0c664de0822e2c"; };

  };

  class ric_calibResponse : public ros::Msg
  {
    public:
      bool ack;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ack;
      u_ack.real = this->ack;
      *(outbuffer + offset + 0) = (u_ack.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ack);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ack;
      u_ack.base = 0;
      u_ack.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ack = u_ack.real;
      offset += sizeof(this->ack);
     return offset;
    }

    const char * getType(){ return RIC_CALIB; };
    const char * getMD5(){ return "8f5729177853f34b146e2e57766d4dc2"; };

  };

  class ric_calib {
    public:
    typedef ric_calibRequest Request;
    typedef ric_calibResponse Response;
  };

}
#endif
