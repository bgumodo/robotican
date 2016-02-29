#ifndef _ROS_dynamixel_msgs_MotorState_h
#define _ROS_dynamixel_msgs_MotorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_msgs
{

  class MotorState : public ros::Msg
  {
    public:
      float timestamp;
      int32_t id;
      int32_t goal;
      int32_t position;
      int32_t error;
      int32_t speed;
      float load;
      float voltage;
      int32_t temperature;
      bool moving;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_timestamp = (int32_t *) &(this->timestamp);
      int32_t exp_timestamp = (((*val_timestamp)>>23)&255);
      if(exp_timestamp != 0)
        exp_timestamp += 1023-127;
      int32_t sig_timestamp = *val_timestamp;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_timestamp<<5) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp>>3) & 0xff;
      *(outbuffer + offset++) = (sig_timestamp>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_timestamp<<4) & 0xF0) | ((sig_timestamp>>19)&0x0F);
      *(outbuffer + offset++) = (exp_timestamp>>4) & 0x7F;
      if(this->timestamp < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_goal;
      u_goal.real = this->goal;
      *(outbuffer + offset + 0) = (u_goal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_goal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_goal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_goal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal);
      union {
        int32_t real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      union {
        int32_t real;
        uint32_t base;
      } u_error;
      u_error.real = this->error;
      *(outbuffer + offset + 0) = (u_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error);
      union {
        int32_t real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      int32_t * val_load = (int32_t *) &(this->load);
      int32_t exp_load = (((*val_load)>>23)&255);
      if(exp_load != 0)
        exp_load += 1023-127;
      int32_t sig_load = *val_load;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_load<<5) & 0xff;
      *(outbuffer + offset++) = (sig_load>>3) & 0xff;
      *(outbuffer + offset++) = (sig_load>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_load<<4) & 0xF0) | ((sig_load>>19)&0x0F);
      *(outbuffer + offset++) = (exp_load>>4) & 0x7F;
      if(this->load < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_voltage = (int32_t *) &(this->voltage);
      int32_t exp_voltage = (((*val_voltage)>>23)&255);
      if(exp_voltage != 0)
        exp_voltage += 1023-127;
      int32_t sig_voltage = *val_voltage;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_voltage<<5) & 0xff;
      *(outbuffer + offset++) = (sig_voltage>>3) & 0xff;
      *(outbuffer + offset++) = (sig_voltage>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_voltage<<4) & 0xF0) | ((sig_voltage>>19)&0x0F);
      *(outbuffer + offset++) = (exp_voltage>>4) & 0x7F;
      if(this->voltage < 0) *(outbuffer + offset -1) |= 0x80;
      union {
        int32_t real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      union {
        bool real;
        uint8_t base;
      } u_moving;
      u_moving.real = this->moving;
      *(outbuffer + offset + 0) = (u_moving.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->moving);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_timestamp = (uint32_t*) &(this->timestamp);
      offset += 3;
      *val_timestamp = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_timestamp |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_timestamp = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_timestamp |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_timestamp !=0)
        *val_timestamp |= ((exp_timestamp)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->timestamp = -this->timestamp;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_goal;
      u_goal.base = 0;
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_goal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->goal = u_goal.real;
      offset += sizeof(this->goal);
      union {
        int32_t real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        int32_t real;
        uint32_t base;
      } u_error;
      u_error.base = 0;
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error = u_error.real;
      offset += sizeof(this->error);
      union {
        int32_t real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      uint32_t * val_load = (uint32_t*) &(this->load);
      offset += 3;
      *val_load = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_load |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_load |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_load |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_load = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_load |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_load !=0)
        *val_load |= ((exp_load)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->load = -this->load;
      uint32_t * val_voltage = (uint32_t*) &(this->voltage);
      offset += 3;
      *val_voltage = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_voltage |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_voltage |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_voltage |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_voltage = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_voltage |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_voltage !=0)
        *val_voltage |= ((exp_voltage)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->voltage = -this->voltage;
      union {
        int32_t real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      union {
        bool real;
        uint8_t base;
      } u_moving;
      u_moving.base = 0;
      u_moving.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->moving = u_moving.real;
      offset += sizeof(this->moving);
     return offset;
    }

    const char * getType(){ return "dynamixel_msgs/MotorState"; };
    const char * getMD5(){ return "1cefdc3ff0c7d52e475886024476b74d"; };

  };

}
#endif