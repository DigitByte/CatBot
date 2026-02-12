#ifndef _ROS_catbot_msgs_eyes_pos_h
#define _ROS_catbot_msgs_eyes_pos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace catbot_msgs
{

  class eyes_pos : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _position_type;
      _position_type position;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;

    eyes_pos():
      position(),
      velocity()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "catbot_msgs/eyes_pos"; };
    virtual const char * getMD5() override { return "589dbd8c358d253de2ab5441acd36a2b"; };

  };

}
#endif
