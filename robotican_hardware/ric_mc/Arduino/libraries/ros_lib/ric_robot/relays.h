#ifndef _ROS_SERVICE_relays_h
#define _ROS_SERVICE_relays_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

static const char RELAYS[] = "ric_robot/relays";

  class relaysRequest : public ros::Msg
  {
    public:
      bool ch1;
      bool ch2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ch1;
      u_ch1.real = this->ch1;
      *(outbuffer + offset + 0) = (u_ch1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ch1);
      union {
        bool real;
        uint8_t base;
      } u_ch2;
      u_ch2.real = this->ch2;
      *(outbuffer + offset + 0) = (u_ch2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ch2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ch1;
      u_ch1.base = 0;
      u_ch1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ch1 = u_ch1.real;
      offset += sizeof(this->ch1);
      union {
        bool real;
        uint8_t base;
      } u_ch2;
      u_ch2.base = 0;
      u_ch2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ch2 = u_ch2.real;
      offset += sizeof(this->ch2);
     return offset;
    }

    const char * getType(){ return RELAYS; };
    const char * getMD5(){ return "1f11a3ed137a90a6662107bc18da38b6"; };

  };

  class relaysResponse : public ros::Msg
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

    const char * getType(){ return RELAYS; };
    const char * getMD5(){ return "8f5729177853f34b146e2e57766d4dc2"; };

  };

  class relays {
    public:
    typedef relaysRequest Request;
    typedef relaysResponse Response;
  };

}
#endif
