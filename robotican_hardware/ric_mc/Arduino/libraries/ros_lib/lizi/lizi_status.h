#ifndef _ROS_lizi_lizi_status_h
#define _ROS_lizi_lizi_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lizi
{

  class lizi_status : public ros::Msg
  {
    public:
      uint8_t faults;
      float battery_voltage;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->faults >> (8 * 0)) & 0xFF;
      offset += sizeof(this->faults);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->faults =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->faults);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
     return offset;
    }

    const char * getType(){ return "lizi/lizi_status"; };
    const char * getMD5(){ return "f66cc2fe91fb70d2b82c88e7c03227df"; };

  };

}
#endif