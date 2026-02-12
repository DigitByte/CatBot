#ifndef _ROS_SERVICE_connect_servos_h
#define _ROS_SERVICE_connect_servos_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace catbot_msgs
{

static const char CONNECT_SERVOS[] = "catbot_msgs/connect_servos";

  class connect_servosRequest : public ros::Msg
  {
    public:
      typedef bool _connect_type;
      _connect_type connect;

    connect_servosRequest():
      connect(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_connect;
      u_connect.real = this->connect;
      *(outbuffer + offset + 0) = (u_connect.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->connect);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_connect;
      u_connect.base = 0;
      u_connect.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->connect = u_connect.real;
      offset += sizeof(this->connect);
     return offset;
    }

    virtual const char * getType() override { return CONNECT_SERVOS; };
    virtual const char * getMD5() override { return "a31bf7076ca051d3bb47d5d808c4982b"; };

  };

  class connect_servosResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    connect_servosResponse():
      result(0)
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
     return offset;
    }

    virtual const char * getType() override { return CONNECT_SERVOS; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class connect_servos {
    public:
    typedef connect_servosRequest Request;
    typedef connect_servosResponse Response;
  };

}
#endif
