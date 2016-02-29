#ifndef _ROS_lizi_lizi_gps_h
#define _ROS_lizi_lizi_gps_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lizi
{

  class lizi_gps : public ros::Msg
  {
    public:
      float Lat;
      float Lon;
      float Alt;
      int16_t Sats;
      int16_t HDOP;
      int16_t Status;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_Lat = (int32_t *) &(this->Lat);
      int32_t exp_Lat = (((*val_Lat)>>23)&255);
      if(exp_Lat != 0)
        exp_Lat += 1023-127;
      int32_t sig_Lat = *val_Lat;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Lat<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Lat>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Lat>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Lat<<4) & 0xF0) | ((sig_Lat>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Lat>>4) & 0x7F;
      if(this->Lat < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_Lon = (int32_t *) &(this->Lon);
      int32_t exp_Lon = (((*val_Lon)>>23)&255);
      if(exp_Lon != 0)
        exp_Lon += 1023-127;
      int32_t sig_Lon = *val_Lon;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_Lon<<5) & 0xff;
      *(outbuffer + offset++) = (sig_Lon>>3) & 0xff;
      *(outbuffer + offset++) = (sig_Lon>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_Lon<<4) & 0xF0) | ((sig_Lon>>19)&0x0F);
      *(outbuffer + offset++) = (exp_Lon>>4) & 0x7F;
      if(this->Lon < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        float real;
        uint32_t base;
      } u_Alt;
      u_Alt.real = this->Alt;
      *(outbuffer + offset + 0) = (u_Alt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Alt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Alt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Alt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Alt);
      union {
        int16_t real;
        uint16_t base;
      } u_Sats;
      u_Sats.real = this->Sats;
      *(outbuffer + offset + 0) = (u_Sats.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Sats.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Sats);
      union {
        int16_t real;
        uint16_t base;
      } u_HDOP;
      u_HDOP.real = this->HDOP;
      *(outbuffer + offset + 0) = (u_HDOP.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_HDOP.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->HDOP);
      union {
        int16_t real;
        uint16_t base;
      } u_Status;
      u_Status.real = this->Status;
      *(outbuffer + offset + 0) = (u_Status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_Lat = (uint32_t*) &(this->Lat);
      offset += 3;
      *val_Lat = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_Lat |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_Lat |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_Lat |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_Lat = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_Lat |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_Lat !=0)
        *val_Lat |= ((exp_Lat)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->Lat = -this->Lat;
      uint32_t * val_Lon = (uint32_t*) &(this->Lon);
      offset += 3;
      *val_Lon = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_Lon |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_Lon |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_Lon |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_Lon = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_Lon |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_Lon !=0)
        *val_Lon |= ((exp_Lon)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->Lon = -this->Lon;
      union {
        float real;
        uint32_t base;
      } u_Alt;
      u_Alt.base = 0;
      u_Alt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Alt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Alt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Alt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Alt = u_Alt.real;
      offset += sizeof(this->Alt);
      union {
        int16_t real;
        uint16_t base;
      } u_Sats;
      u_Sats.base = 0;
      u_Sats.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Sats.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Sats = u_Sats.real;
      offset += sizeof(this->Sats);
      union {
        int16_t real;
        uint16_t base;
      } u_HDOP;
      u_HDOP.base = 0;
      u_HDOP.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_HDOP.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->HDOP = u_HDOP.real;
      offset += sizeof(this->HDOP);
      union {
        int16_t real;
        uint16_t base;
      } u_Status;
      u_Status.base = 0;
      u_Status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Status = u_Status.real;
      offset += sizeof(this->Status);
     return offset;
    }

    const char * getType(){ return "lizi/lizi_gps"; };
    const char * getMD5(){ return "b6b3576ebfcb04caffd96f3cf662f564"; };

  };

}
#endif