#ifndef _ROS_SERVICE_set_elevator_h
#define _ROS_SERVICE_set_elevator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

static const char SET_ELEVATOR[] = "ric_robot/set_elevator";

  class set_elevatorRequest : public ros::Msg
  {
    public:
      float pos;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pos;
      u_pos.real = this->pos;
      *(outbuffer + offset + 0) = (u_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pos;
      u_pos.base = 0;
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos = u_pos.real;
      offset += sizeof(this->pos);
     return offset;
    }

    const char * getType(){ return SET_ELEVATOR; };
    const char * getMD5(){ return "b6fb6507bc71350dd1c10d16c76b741e"; };

  };

  class set_elevatorResponse : public ros::Msg
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

    const char * getType(){ return SET_ELEVATOR; };
    const char * getMD5(){ return "8f5729177853f34b146e2e57766d4dc2"; };

  };

  class set_elevator {
    public:
    typedef set_elevatorRequest Request;
    typedef set_elevatorResponse Response;
  };

}
#endif
