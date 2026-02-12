#ifndef _ROS_catbot_msgs_contact_detection_h
#define _ROS_catbot_msgs_contact_detection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace catbot_msgs
{

  class contact_detection : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t feet_stance_length;
      typedef bool _feet_stance_type;
      _feet_stance_type st_feet_stance;
      _feet_stance_type * feet_stance;

    contact_detection():
      header(),
      feet_stance_length(0), st_feet_stance(), feet_stance(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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

    virtual const char * getType() override { return "catbot_msgs/contact_detection"; };
    virtual const char * getMD5() override { return "b9731897385d5a63ef24c7710bbc7fd1"; };

  };

}
#endif
