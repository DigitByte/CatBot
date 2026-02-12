#ifndef _ROS_catbot_msgs_feet_pressure_h
#define _ROS_catbot_msgs_feet_pressure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace catbot_msgs
{

  class feet_pressure : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t leg_name_length;
      typedef char* _leg_name_type;
      _leg_name_type st_leg_name;
      _leg_name_type * leg_name;
      uint32_t pressure_vector_length;
      typedef geometry_msgs::Point _pressure_vector_type;
      _pressure_vector_type st_pressure_vector;
      _pressure_vector_type * pressure_vector;
      uint32_t feet_pos_length;
      typedef geometry_msgs::Point _feet_pos_type;
      _feet_pos_type st_feet_pos;
      _feet_pos_type * feet_pos;
      uint32_t feet_stance_length;
      typedef bool _feet_stance_type;
      _feet_stance_type st_feet_stance;
      _feet_stance_type * feet_stance;

    feet_pressure():
      header(),
      leg_name_length(0), st_leg_name(), leg_name(nullptr),
      pressure_vector_length(0), st_pressure_vector(), pressure_vector(nullptr),
      feet_pos_length(0), st_feet_pos(), feet_pos(nullptr),
      feet_stance_length(0), st_feet_stance(), feet_stance(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->leg_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->leg_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->leg_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->leg_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leg_name_length);
      for( uint32_t i = 0; i < leg_name_length; i++){
      uint32_t length_leg_namei = strlen(this->leg_name[i]);
      varToArr(outbuffer + offset, length_leg_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->leg_name[i], length_leg_namei);
      offset += length_leg_namei;
      }
      *(outbuffer + offset + 0) = (this->pressure_vector_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pressure_vector_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pressure_vector_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pressure_vector_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pressure_vector_length);
      for( uint32_t i = 0; i < pressure_vector_length; i++){
      offset += this->pressure_vector[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->feet_pos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->feet_pos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->feet_pos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->feet_pos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->feet_pos_length);
      for( uint32_t i = 0; i < feet_pos_length; i++){
      offset += this->feet_pos[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->feet_stance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->feet_stance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->feet_stance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->feet_stance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->feet_stance_length);
      for( uint32_t i = 0; i < feet_stance_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_feet_stancei;
      u_feet_stancei.real = this->feet_stance[i];
      *(outbuffer + offset + 0) = (u_feet_stancei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->feet_stance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t leg_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      leg_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      leg_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      leg_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->leg_name_length);
      if(leg_name_lengthT > leg_name_length)
        this->leg_name = (char**)realloc(this->leg_name, leg_name_lengthT * sizeof(char*));
      leg_name_length = leg_name_lengthT;
      for( uint32_t i = 0; i < leg_name_length; i++){
      uint32_t length_st_leg_name;
      arrToVar(length_st_leg_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_leg_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_leg_name-1]=0;
      this->st_leg_name = (char *)(inbuffer + offset-1);
      offset += length_st_leg_name;
        memcpy( &(this->leg_name[i]), &(this->st_leg_name), sizeof(char*));
      }
      uint32_t pressure_vector_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pressure_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pressure_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pressure_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pressure_vector_length);
      if(pressure_vector_lengthT > pressure_vector_length)
        this->pressure_vector = (geometry_msgs::Point*)realloc(this->pressure_vector, pressure_vector_lengthT * sizeof(geometry_msgs::Point));
      pressure_vector_length = pressure_vector_lengthT;
      for( uint32_t i = 0; i < pressure_vector_length; i++){
      offset += this->st_pressure_vector.deserialize(inbuffer + offset);
        memcpy( &(this->pressure_vector[i]), &(this->st_pressure_vector), sizeof(geometry_msgs::Point));
      }
      uint32_t feet_pos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      feet_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      feet_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      feet_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->feet_pos_length);
      if(feet_pos_lengthT > feet_pos_length)
        this->feet_pos = (geometry_msgs::Point*)realloc(this->feet_pos, feet_pos_lengthT * sizeof(geometry_msgs::Point));
      feet_pos_length = feet_pos_lengthT;
      for( uint32_t i = 0; i < feet_pos_length; i++){
      offset += this->st_feet_pos.deserialize(inbuffer + offset);
        memcpy( &(this->feet_pos[i]), &(this->st_feet_pos), sizeof(geometry_msgs::Point));
      }
      uint32_t feet_stance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      feet_stance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      feet_stance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      feet_stance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->feet_stance_length);
      if(feet_stance_lengthT > feet_stance_length)
        this->feet_stance = (bool*)realloc(this->feet_stance, feet_stance_lengthT * sizeof(bool));
      feet_stance_length = feet_stance_lengthT;
      for( uint32_t i = 0; i < feet_stance_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_feet_stance;
      u_st_feet_stance.base = 0;
      u_st_feet_stance.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_feet_stance = u_st_feet_stance.real;
      offset += sizeof(this->st_feet_stance);
        memcpy( &(this->feet_stance[i]), &(this->st_feet_stance), sizeof(bool));
      }
     return offset;
    }

    virtual const char * getType() override { return "catbot_msgs/feet_pressure"; };
    virtual const char * getMD5() override { return "30847eb632b3914317fa2e135cd9287a"; };

  };

}
#endif
