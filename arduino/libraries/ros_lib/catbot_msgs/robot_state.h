#ifndef _ROS_catbot_msgs_robot_state_h
#define _ROS_catbot_msgs_robot_state_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace catbot_msgs
{

  class robot_state : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _linear_velocity_type;
      _linear_velocity_type linear_velocity;
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      typedef geometry_msgs::Vector3 _feet_fl_position_type;
      _feet_fl_position_type feet_fl_position;
      typedef geometry_msgs::Vector3 _feet_fr_position_type;
      _feet_fr_position_type feet_fr_position;
      typedef geometry_msgs::Vector3 _feet_bl_position_type;
      _feet_bl_position_type feet_bl_position;
      typedef geometry_msgs::Vector3 _feet_br_position_type;
      _feet_br_position_type feet_br_position;

    robot_state():
      header(),
      pose(),
      angular_velocity(),
      linear_velocity(),
      linear_acceleration(),
      feet_fl_position(),
      feet_fr_position(),
      feet_bl_position(),
      feet_br_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->feet_fl_position.serialize(outbuffer + offset);
      offset += this->feet_fr_position.serialize(outbuffer + offset);
      offset += this->feet_bl_position.serialize(outbuffer + offset);
      offset += this->feet_br_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->feet_fl_position.deserialize(inbuffer + offset);
      offset += this->feet_fr_position.deserialize(inbuffer + offset);
      offset += this->feet_bl_position.deserialize(inbuffer + offset);
      offset += this->feet_br_position.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "catbot_msgs/robot_state"; };
    virtual const char * getMD5() override { return "d00fcf8a1266c18340ee5163d127a2db"; };

  };

}
#endif
