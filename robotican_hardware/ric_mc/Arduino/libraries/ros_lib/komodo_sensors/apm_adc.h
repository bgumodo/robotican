#ifndef _ROS_komodo_sensors_apm_adc_h
#define _ROS_komodo_sensors_apm_adc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace komodo_sensors
{

  class apm_adc : public ros::Msg
  {
    public:
      uint16_t A0;
      uint16_t A1;
      uint16_t A2;
      uint16_t A3;
      uint16_t A4;
      uint16_t A5;
      uint16_t A6;
      uint16_t A7;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->A0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A0);
      *(outbuffer + offset + 0) = (this->A1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A1);
      *(outbuffer + offset + 0) = (this->A2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A2);
      *(outbuffer + offset + 0) = (this->A3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A3);
      *(outbuffer + offset + 0) = (this->A4 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A4 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A4);
      *(outbuffer + offset + 0) = (this->A5 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A5 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A5);
      *(outbuffer + offset + 0) = (this->A6 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A6 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A6);
      *(outbuffer + offset + 0) = (this->A7 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->A7 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->A7);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->A0 =  ((uint16_t) (*(inbuffer + offset)));
      this->A0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A0);
      this->A1 =  ((uint16_t) (*(inbuffer + offset)));
      this->A1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A1);
      this->A2 =  ((uint16_t) (*(inbuffer + offset)));
      this->A2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A2);
      this->A3 =  ((uint16_t) (*(inbuffer + offset)));
      this->A3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A3);
      this->A4 =  ((uint16_t) (*(inbuffer + offset)));
      this->A4 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A4);
      this->A5 =  ((uint16_t) (*(inbuffer + offset)));
      this->A5 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A5);
      this->A6 =  ((uint16_t) (*(inbuffer + offset)));
      this->A6 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A6);
      this->A7 =  ((uint16_t) (*(inbuffer + offset)));
      this->A7 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->A7);
     return offset;
    }

    const char * getType(){ return "komodo_sensors/apm_adc"; };
    const char * getMD5(){ return "5d009541046908a1e7a9f62e0bb7a56e"; };

  };

}
#endif