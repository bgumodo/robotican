#ifndef _ROS_dynamixel_msgs_JointState_h
#define _ROS_dynamixel_msgs_JointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dynamixel_msgs
{

  class JointState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* name;
      uint8_t motor_ids_length;
      int32_t st_motor_ids;
      int32_t * motor_ids;
      uint8_t motor_temps_length;
      int32_t st_motor_temps;
      int32_t * motor_temps;
      float goal_pos;
      float current_pos;
      float error;
      float velocity;
      float load;
      bool is_moving;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset++) = motor_ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < motor_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_motor_idsi;
      u_motor_idsi.real = this->motor_ids[i];
      *(outbuffer + offset + 0) = (u_motor_idsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_idsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_idsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_idsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_ids[i]);
      }
      *(outbuffer + offset++) = motor_temps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < motor_temps_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_motor_tempsi;
      u_motor_tempsi.real = this->motor_temps[i];
      *(outbuffer + offset + 0) = (u_motor_tempsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_tempsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_tempsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_tempsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_temps[i]);
      }
      int32_t * val_goal_pos = (int32_t *) &(this->goal_pos);
      int32_t exp_goal_pos = (((*val_goal_pos)>>23)&255);
      if(exp_goal_pos != 0)
        exp_goal_pos += 1023-127;
      int32_t sig_goal_pos = *val_goal_pos;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_goal_pos<<5) & 0xff;
      *(outbuffer + offset++) = (sig_goal_pos>>3) & 0xff;
      *(outbuffer + offset++) = (sig_goal_pos>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_goal_pos<<4) & 0xF0) | ((sig_goal_pos>>19)&0x0F);
      *(outbuffer + offset++) = (exp_goal_pos>>4) & 0x7F;
      if(this->goal_pos < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_current_pos = (int32_t *) &(this->current_pos);
      int32_t exp_current_pos = (((*val_current_pos)>>23)&255);
      if(exp_current_pos != 0)
        exp_current_pos += 1023-127;
      int32_t sig_current_pos = *val_current_pos;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_current_pos<<5) & 0xff;
      *(outbuffer + offset++) = (sig_current_pos>>3) & 0xff;
      *(outbuffer + offset++) = (sig_current_pos>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_current_pos<<4) & 0xF0) | ((sig_current_pos>>19)&0x0F);
      *(outbuffer + offset++) = (exp_current_pos>>4) & 0x7F;
      if(this->current_pos < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_error = (int32_t *) &(this->error);
      int32_t exp_error = (((*val_error)>>23)&255);
      if(exp_error != 0)
        exp_error += 1023-127;
      int32_t sig_error = *val_error;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_error<<5) & 0xff;
      *(outbuffer + offset++) = (sig_error>>3) & 0xff;
      *(outbuffer + offset++) = (sig_error>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_error<<4) & 0xF0) | ((sig_error>>19)&0x0F);
      *(outbuffer + offset++) = (exp_error>>4) & 0x7F;
      if(this->error < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_velocity = (int32_t *) &(this->velocity);
      int32_t exp_velocity = (((*val_velocity)>>23)&255);
      if(exp_velocity != 0)
        exp_velocity += 1023-127;
      int32_t sig_velocity = *val_velocity;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_velocity<<5) & 0xff;
      *(outbuffer + offset++) = (sig_velocity>>3) & 0xff;
      *(outbuffer + offset++) = (sig_velocity>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_velocity<<4) & 0xF0) | ((sig_velocity>>19)&0x0F);
      *(outbuffer + offset++) = (exp_velocity>>4) & 0x7F;
      if(this->velocity < 0) *(outbuffer + offset -1) |= 0x80;
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
      union {
        bool real;
        uint8_t base;
      } u_is_moving;
      u_is_moving.real = this->is_moving;
      *(outbuffer + offset + 0) = (u_is_moving.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_moving);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint8_t motor_ids_lengthT = *(inbuffer + offset++);
      if(motor_ids_lengthT > motor_ids_length)
        this->motor_ids = (int32_t*)realloc(this->motor_ids, motor_ids_lengthT * sizeof(int32_t));
      offset += 3;
      motor_ids_length = motor_ids_lengthT;
      for( uint8_t i = 0; i < motor_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_motor_ids;
      u_st_motor_ids.base = 0;
      u_st_motor_ids.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_motor_ids.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_motor_ids.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_motor_ids.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_motor_ids = u_st_motor_ids.real;
      offset += sizeof(this->st_motor_ids);
        memcpy( &(this->motor_ids[i]), &(this->st_motor_ids), sizeof(int32_t));
      }
      uint8_t motor_temps_lengthT = *(inbuffer + offset++);
      if(motor_temps_lengthT > motor_temps_length)
        this->motor_temps = (int32_t*)realloc(this->motor_temps, motor_temps_lengthT * sizeof(int32_t));
      offset += 3;
      motor_temps_length = motor_temps_lengthT;
      for( uint8_t i = 0; i < motor_temps_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_motor_temps;
      u_st_motor_temps.base = 0;
      u_st_motor_temps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_motor_temps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_motor_temps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_motor_temps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_motor_temps = u_st_motor_temps.real;
      offset += sizeof(this->st_motor_temps);
        memcpy( &(this->motor_temps[i]), &(this->st_motor_temps), sizeof(int32_t));
      }
      uint32_t * val_goal_pos = (uint32_t*) &(this->goal_pos);
      offset += 3;
      *val_goal_pos = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_goal_pos |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_goal_pos |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_goal_pos |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_goal_pos = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_goal_pos |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_goal_pos !=0)
        *val_goal_pos |= ((exp_goal_pos)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->goal_pos = -this->goal_pos;
      uint32_t * val_current_pos = (uint32_t*) &(this->current_pos);
      offset += 3;
      *val_current_pos = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_current_pos |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_current_pos |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_current_pos |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_current_pos = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_current_pos |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_current_pos !=0)
        *val_current_pos |= ((exp_current_pos)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->current_pos = -this->current_pos;
      uint32_t * val_error = (uint32_t*) &(this->error);
      offset += 3;
      *val_error = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_error |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_error |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_error |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_error = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_error |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_error !=0)
        *val_error |= ((exp_error)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->error = -this->error;
      uint32_t * val_velocity = (uint32_t*) &(this->velocity);
      offset += 3;
      *val_velocity = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_velocity |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_velocity |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_velocity |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_velocity = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_velocity |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_velocity !=0)
        *val_velocity |= ((exp_velocity)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->velocity = -this->velocity;
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
      union {
        bool real;
        uint8_t base;
      } u_is_moving;
      u_is_moving.base = 0;
      u_is_moving.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_moving = u_is_moving.real;
      offset += sizeof(this->is_moving);
     return offset;
    }

    const char * getType(){ return "dynamixel_msgs/JointState"; };
    const char * getMD5(){ return "2b8449320cde76616338e2539db27c32"; };

  };

}
#endif