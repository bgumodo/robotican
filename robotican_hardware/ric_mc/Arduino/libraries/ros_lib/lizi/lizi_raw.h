#ifndef _ROS_lizi_lizi_raw_h
#define _ROS_lizi_lizi_raw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lizi
{

  class lizi_raw : public ros::Msg
  {
    public:
      float qw;
      float qx;
      float qy;
      float qz;
      int32_t left_ticks;
      int32_t right_ticks;
      float left_urf;
      float rear_urf;
      float right_urf;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_qw;
      u_qw.real = this->qw;
      *(outbuffer + offset + 0) = (u_qw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qw);
      union {
        float real;
        uint32_t base;
      } u_qx;
      u_qx.real = this->qx;
      *(outbuffer + offset + 0) = (u_qx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qx);
      union {
        float real;
        uint32_t base;
      } u_qy;
      u_qy.real = this->qy;
      *(outbuffer + offset + 0) = (u_qy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qy);
      union {
        float real;
        uint32_t base;
      } u_qz;
      u_qz.real = this->qz;
      *(outbuffer + offset + 0) = (u_qz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qz);
      union {
        int32_t real;
        uint32_t base;
      } u_left_ticks;
      u_left_ticks.real = this->left_ticks;
      *(outbuffer + offset + 0) = (u_left_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_ticks.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_ticks.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_ticks.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_ticks);
      union {
        int32_t real;
        uint32_t base;
      } u_right_ticks;
      u_right_ticks.real = this->right_ticks;
      *(outbuffer + offset + 0) = (u_right_ticks.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_ticks.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_ticks.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_ticks.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_ticks);
      union {
        float real;
        uint32_t base;
      } u_left_urf;
      u_left_urf.real = this->left_urf;
      *(outbuffer + offset + 0) = (u_left_urf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_urf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_urf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_urf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_urf);
      union {
        float real;
        uint32_t base;
      } u_rear_urf;
      u_rear_urf.real = this->rear_urf;
      *(outbuffer + offset + 0) = (u_rear_urf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_urf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_urf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_urf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_urf);
      union {
        float real;
        uint32_t base;
      } u_right_urf;
      u_right_urf.real = this->right_urf;
      *(outbuffer + offset + 0) = (u_right_urf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_urf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_urf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_urf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_urf);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_qw;
      u_qw.base = 0;
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qw = u_qw.real;
      offset += sizeof(this->qw);
      union {
        float real;
        uint32_t base;
      } u_qx;
      u_qx.base = 0;
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qx = u_qx.real;
      offset += sizeof(this->qx);
      union {
        float real;
        uint32_t base;
      } u_qy;
      u_qy.base = 0;
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qy = u_qy.real;
      offset += sizeof(this->qy);
      union {
        float real;
        uint32_t base;
      } u_qz;
      u_qz.base = 0;
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qz = u_qz.real;
      offset += sizeof(this->qz);
      union {
        int32_t real;
        uint32_t base;
      } u_left_ticks;
      u_left_ticks.base = 0;
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_ticks.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_ticks = u_left_ticks.real;
      offset += sizeof(this->left_ticks);
      union {
        int32_t real;
        uint32_t base;
      } u_right_ticks;
      u_right_ticks.base = 0;
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_ticks.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_ticks = u_right_ticks.real;
      offset += sizeof(this->right_ticks);
      union {
        float real;
        uint32_t base;
      } u_left_urf;
      u_left_urf.base = 0;
      u_left_urf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_urf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_urf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_urf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_urf = u_left_urf.real;
      offset += sizeof(this->left_urf);
      union {
        float real;
        uint32_t base;
      } u_rear_urf;
      u_rear_urf.base = 0;
      u_rear_urf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_urf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_urf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_urf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_urf = u_rear_urf.real;
      offset += sizeof(this->rear_urf);
      union {
        float real;
        uint32_t base;
      } u_right_urf;
      u_right_urf.base = 0;
      u_right_urf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_urf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_urf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_urf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_urf = u_right_urf.real;
      offset += sizeof(this->right_urf);
     return offset;
    }

    const char * getType(){ return "lizi/lizi_raw"; };
    const char * getMD5(){ return "d3fede5537f79f19050f2e81db4deb6d"; };

  };

}
#endif