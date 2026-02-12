#ifndef _ROS_SERVICE_set_gait_h
#define _ROS_SERVICE_set_gait_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace catbot_msgs
{

static const char SET_GAIT[] = "catbot_msgs/set_gait";

  class set_gaitRequest : public ros::Msg
  {
    public:
      typedef const char* _gait_filename_type;
      _gait_filename_type gait_filename;

    set_gaitRequest():
      gait_filename("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_gait_filename = strlen(this->gait_filename);
      varToArr(outbuffer + offset, length_gait_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->gait_filename, length_gait_filename);
      offset += length_gait_filename;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_gait_filename;
      arrToVar(length_gait_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_gait_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_gait_filename-1]=0;
      this->gait_filename = (char *)(inbuffer + offset-1);
      offset += length_gait_filename;
     return offset;
    }

    virtual const char * getType() override { return SET_GAIT; };
    virtual const char * getMD5() override { return "748de5a12429f1dcd60c676528b23aa2"; };

  };

  class set_gaitResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;
      typedef const char* _info_type;
      _info_type info;

    set_gaitResponse():
      result(0),
      info("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      uint32_t length_info = strlen(this->info);
      varToArr(outbuffer + offset, length_info);
      offset += 4;
      memcpy(outbuffer + offset, this->info, length_info);
      offset += length_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
      uint32_t length_info;
      arrToVar(length_info, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_info-1]=0;
      this->info = (char *)(inbuffer + offset-1);
      offset += length_info;
     return offset;
    }

    virtual const char * getType() override { return SET_GAIT; };
    virtual const char * getMD5() override { return "929b8c0d7b68510a3f501a60258c746e"; };

  };

  class set_gait {
    public:
    typedef set_gaitRequest Request;
    typedef set_gaitResponse Response;
  };

}
#endif
