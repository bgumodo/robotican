#ifndef _ROS_SERVICE_home_elevator_h
#define _ROS_SERVICE_home_elevator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

static const char HOME_ELEVATOR[] = "ric_robot/home_elevator";

  class home_elevatorRequest : public ros::Msg
  {
    public:
      int8_t dir;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_dir;
      u_dir.real = this->dir;
      *(outbuffer + offset + 0) = (u_dir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_dir;
      u_dir.base = 0;
      u_dir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dir = u_dir.real;
      offset += sizeof(this->dir);
     return offset;
    }

    const char * getType(){ return HOME_ELEVATOR; };
    const char * getMD5(){ return "1cf84cfbab276e0cb1fa7c3b384eb912"; };

  };

  class home_elevatorResponse : public ros::Msg
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

    const char * getType(){ return HOME_ELEVATOR; };
    const char * getMD5(){ return "8f5729177853f34b146e2e57766d4dc2"; };

  };

  class home_elevator {
    public:
    typedef home_elevatorRequest Request;
    typedef home_elevatorResponse Response;
  };

}
#endif
