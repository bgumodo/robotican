#ifndef _ROS_komodo_sensors_apm_gps_h
#define _ROS_komodo_sensors_apm_gps_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace komodo_sensors
{

  class apm_gps : public ros::Msg
  {
    public:
      int32_t lat;
      int32_t lon;
      int16_t alt;
      int16_t sats;
      int16_t hdop;
      int16_t status;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_lat;
      u_lat.real = this->lat;
      *(outbuffer + offset + 0) = (u_lat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lat.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lat);
      union {
        int32_t real;
        uint32_t base;
      } u_lon;
      u_lon.real = this->lon;
      *(outbuffer + offset + 0) = (u_lon.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lon.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lon.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lon.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lon);
      union {
        int16_t real;
        uint16_t base;
      } u_alt;
      u_alt.real = this->alt;
      *(outbuffer + offset + 0) = (u_alt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_alt.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->alt);
      union {
        int16_t real;
        uint16_t base;
      } u_sats;
      u_sats.real = this->sats;
      *(outbuffer + offset + 0) = (u_sats.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sats.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sats);
      union {
        int16_t real;
        uint16_t base;
      } u_hdop;
      u_hdop.real = this->hdop;
      *(outbuffer + offset + 0) = (u_hdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hdop.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hdop);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_lat;
      u_lat.base = 0;
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lat = u_lat.real;
      offset += sizeof(this->lat);
      union {
        int32_t real;
        uint32_t base;
      } u_lon;
      u_lon.base = 0;
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lon = u_lon.real;
      offset += sizeof(this->lon);
      union {
        int16_t real;
        uint16_t base;
      } u_alt;
      u_alt.base = 0;
      u_alt.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_alt.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->alt = u_alt.real;
      offset += sizeof(this->alt);
      union {
        int16_t real;
        uint16_t base;
      } u_sats;
      u_sats.base = 0;
      u_sats.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sats.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sats = u_sats.real;
      offset += sizeof(this->sats);
      union {
        int16_t real;
        uint16_t base;
      } u_hdop;
      u_hdop.base = 0;
      u_hdop.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hdop.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hdop = u_hdop.real;
      offset += sizeof(this->hdop);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "komodo_sensors/apm_gps"; };
    const char * getMD5(){ return "c993a66a6584fc0d2871154e12be7053"; };

  };

}
#endif