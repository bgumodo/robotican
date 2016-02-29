#ifndef _ROS_komodo_sensors_apm_imu_h
#define _ROS_komodo_sensors_apm_imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace komodo_sensors
{

  class apm_imu : public ros::Msg
  {
    public:
      float ax;
      float ay;
      float az;
      float gx;
      float gy;
      float gz;
      float roll;
      float pitch;
      float yaw;
      float heading;
      float mx;
      float my;
      float mz;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ax;
      u_ax.real = this->ax;
      *(outbuffer + offset + 0) = (u_ax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ax);
      union {
        float real;
        uint32_t base;
      } u_ay;
      u_ay.real = this->ay;
      *(outbuffer + offset + 0) = (u_ay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ay);
      union {
        float real;
        uint32_t base;
      } u_az;
      u_az.real = this->az;
      *(outbuffer + offset + 0) = (u_az.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_az.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_az.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_az.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->az);
      union {
        float real;
        uint32_t base;
      } u_gx;
      u_gx.real = this->gx;
      *(outbuffer + offset + 0) = (u_gx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gx);
      union {
        float real;
        uint32_t base;
      } u_gy;
      u_gy.real = this->gy;
      *(outbuffer + offset + 0) = (u_gy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gy);
      union {
        float real;
        uint32_t base;
      } u_gz;
      u_gz.real = this->gz;
      *(outbuffer + offset + 0) = (u_gz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gz);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_mx;
      u_mx.real = this->mx;
      *(outbuffer + offset + 0) = (u_mx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mx);
      union {
        float real;
        uint32_t base;
      } u_my;
      u_my.real = this->my;
      *(outbuffer + offset + 0) = (u_my.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_my.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_my.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_my.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->my);
      union {
        float real;
        uint32_t base;
      } u_mz;
      u_mz.real = this->mz;
      *(outbuffer + offset + 0) = (u_mz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_ax;
      u_ax.base = 0;
      u_ax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ax = u_ax.real;
      offset += sizeof(this->ax);
      union {
        float real;
        uint32_t base;
      } u_ay;
      u_ay.base = 0;
      u_ay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ay = u_ay.real;
      offset += sizeof(this->ay);
      union {
        float real;
        uint32_t base;
      } u_az;
      u_az.base = 0;
      u_az.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_az.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_az.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_az.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->az = u_az.real;
      offset += sizeof(this->az);
      union {
        float real;
        uint32_t base;
      } u_gx;
      u_gx.base = 0;
      u_gx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gx = u_gx.real;
      offset += sizeof(this->gx);
      union {
        float real;
        uint32_t base;
      } u_gy;
      u_gy.base = 0;
      u_gy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gy = u_gy.real;
      offset += sizeof(this->gy);
      union {
        float real;
        uint32_t base;
      } u_gz;
      u_gz.base = 0;
      u_gz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gz = u_gz.real;
      offset += sizeof(this->gz);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      union {
        float real;
        uint32_t base;
      } u_mx;
      u_mx.base = 0;
      u_mx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mx = u_mx.real;
      offset += sizeof(this->mx);
      union {
        float real;
        uint32_t base;
      } u_my;
      u_my.base = 0;
      u_my.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_my.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_my.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_my.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->my = u_my.real;
      offset += sizeof(this->my);
      union {
        float real;
        uint32_t base;
      } u_mz;
      u_mz.base = 0;
      u_mz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mz = u_mz.real;
      offset += sizeof(this->mz);
     return offset;
    }

    const char * getType(){ return "komodo_sensors/apm_imu"; };
    const char * getMD5(){ return "13f07ef07721826d5bb32ddaf22559ad"; };

  };

}
#endif