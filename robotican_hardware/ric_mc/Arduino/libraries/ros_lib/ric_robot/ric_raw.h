#ifndef _ROS_ric_robot_ric_raw_h
#define _ROS_ric_robot_ric_raw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ric_robot
{

  class ric_raw : public ros::Msg
  {
    public:
      float orientation[4];
      int16_t linear_acceleration[3];
      int16_t angular_velocity[3];
      int16_t magnetometer[3];
      int32_t encoders[2];
      float urf[3];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_orientationi;
      u_orientationi.real = this->orientation[i];
      *(outbuffer + offset + 0) = (u_orientationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientationi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientationi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientationi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_linear_accelerationi;
      u_linear_accelerationi.real = this->linear_acceleration[i];
      *(outbuffer + offset + 0) = (u_linear_accelerationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_accelerationi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->linear_acceleration[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_angular_velocityi;
      u_angular_velocityi.real = this->angular_velocity[i];
      *(outbuffer + offset + 0) = (u_angular_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocityi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angular_velocity[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_magnetometeri;
      u_magnetometeri.real = this->magnetometer[i];
      *(outbuffer + offset + 0) = (u_magnetometeri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_magnetometeri.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->magnetometer[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_encodersi;
      u_encodersi.real = this->encoders[i];
      *(outbuffer + offset + 0) = (u_encodersi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encodersi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encodersi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encodersi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoders[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_urfi;
      u_urfi.real = this->urf[i];
      *(outbuffer + offset + 0) = (u_urfi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_urfi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_urfi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_urfi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->urf[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_orientationi;
      u_orientationi.base = 0;
      u_orientationi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientationi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientationi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientationi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orientation[i] = u_orientationi.real;
      offset += sizeof(this->orientation[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_linear_accelerationi;
      u_linear_accelerationi.base = 0;
      u_linear_accelerationi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_accelerationi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->linear_acceleration[i] = u_linear_accelerationi.real;
      offset += sizeof(this->linear_acceleration[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_angular_velocityi;
      u_angular_velocityi.base = 0;
      u_angular_velocityi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocityi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angular_velocity[i] = u_angular_velocityi.real;
      offset += sizeof(this->angular_velocity[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_magnetometeri;
      u_magnetometeri.base = 0;
      u_magnetometeri.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_magnetometeri.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magnetometer[i] = u_magnetometeri.real;
      offset += sizeof(this->magnetometer[i]);
      }
      for( uint8_t i = 0; i < 2; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_encodersi;
      u_encodersi.base = 0;
      u_encodersi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encodersi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encodersi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encodersi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoders[i] = u_encodersi.real;
      offset += sizeof(this->encoders[i]);
      }
      for( uint8_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_urfi;
      u_urfi.base = 0;
      u_urfi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_urfi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_urfi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_urfi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->urf[i] = u_urfi.real;
      offset += sizeof(this->urf[i]);
      }
     return offset;
    }

    const char * getType(){ return "ric_robot/ric_raw"; };
    const char * getMD5(){ return "717e58ef32c83e8c93ceae03d4826367"; };

  };

}
#endif