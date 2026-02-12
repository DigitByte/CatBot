#ifndef _ROS_SERVICE_set_face_h
#define _ROS_SERVICE_set_face_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace catbot_msgs
{

static const char SET_FACE[] = "catbot_msgs/set_face";

  class set_faceRequest : public ros::Msg
  {
    public:
      typedef const char* _style_type;
      _style_type style;
      typedef float _blink_prob_type;
      _blink_prob_type blink_prob;

    set_faceRequest():
      style(""),
      blink_prob(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_style = strlen(this->style);
      varToArr(outbuffer + offset, length_style);
      offset += 4;
      memcpy(outbuffer + offset, this->style, length_style);
      offset += length_style;
      union {
        float real;
        uint32_t base;
      } u_blink_prob;
      u_blink_prob.real = this->blink_prob;
      *(outbuffer + offset + 0) = (u_blink_prob.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blink_prob.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blink_prob.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blink_prob.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blink_prob);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_style;
      arrToVar(length_style, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_style; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_style-1]=0;
      this->style = (char *)(inbuffer + offset-1);
      offset += length_style;
      union {
        float real;
        uint32_t base;
      } u_blink_prob;
      u_blink_prob.base = 0;
      u_blink_prob.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blink_prob.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blink_prob.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blink_prob.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->blink_prob = u_blink_prob.real;
      offset += sizeof(this->blink_prob);
     return offset;
    }

    virtual const char * getType() override { return SET_FACE; };
    virtual const char * getMD5() override { return "ec7fd37ba165f460fe63840aafdb14b4"; };

  };

  class set_faceResponse : public ros::Msg
  {
    public:

    set_faceResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return SET_FACE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class set_face {
    public:
    typedef set_faceRequest Request;
    typedef set_faceResponse Response;
  };

}
#endif
