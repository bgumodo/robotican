#ifndef _ROS_SERVICE_imu_calib_h
#define _ROS_SERVICE_imu_calib_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lizi
{

static const char IMU_CALIB[] = "lizi/imu_calib";

  class imu_calibRequest : public ros::Msg
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

    const char * getType(){ return IMU_CALIB; };
    const char * getMD5(){ return "0dce049a176069675a0c664de0822e2c"; };

  };

  class imu_calibResponse : public ros::Msg
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

    const char * getType(){ return IMU_CALIB; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class imu_calib {
    public:
    typedef imu_calibRequest Request;
    typedef imu_calibResponse Response;
  };

}
#endif
