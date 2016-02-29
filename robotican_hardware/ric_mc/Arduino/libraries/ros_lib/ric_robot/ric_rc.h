#ifndef _ROS_ric_robot_ric_rc_h
#define _ROS_ric_robot_ric_rc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

  class ric_rc : public ros::Msg
  {
    public:
      uint16_t RX1;
      uint16_t RX2;
      uint16_t RX3;
      uint16_t RX4;
      uint16_t RX5;
      uint16_t RX6;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->RX1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RX1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RX1);
      *(outbuffer + offset + 0) = (this->RX2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RX2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RX2);
      *(outbuffer + offset + 0) = (this->RX3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RX3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RX3);
      *(outbuffer + offset + 0) = (this->RX4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RX4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RX4);
      *(outbuffer + offset + 0) = (this->RX5 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RX5 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RX5);
      *(outbuffer + offset + 0) = (this->RX6 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RX6 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RX6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->RX1 =  ((uint16_t) (*(inbuffer + offset)));
      this->RX1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RX1);
      this->RX2 =  ((uint16_t) (*(inbuffer + offset)));
      this->RX2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RX2);
      this->RX3 =  ((uint16_t) (*(inbuffer + offset)));
      this->RX3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RX3);
      this->RX4 =  ((uint16_t) (*(inbuffer + offset)));
      this->RX4 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RX4);
      this->RX5 =  ((uint16_t) (*(inbuffer + offset)));
      this->RX5 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RX5);
      this->RX6 =  ((uint16_t) (*(inbuffer + offset)));
      this->RX6 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RX6);
     return offset;
    }

    const char * getType(){ return "ric_robot/ric_rc"; };
    const char * getMD5(){ return "141c3eb3d7391c30d945399a5b34b7d1"; };

  };

}
#endif