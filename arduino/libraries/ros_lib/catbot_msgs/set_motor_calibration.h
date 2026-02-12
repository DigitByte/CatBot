#ifndef _ROS_SERVICE_set_motor_calibration_h
#define _ROS_SERVICE_set_motor_calibration_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace catbot_msgs
{

static const char SET_MOTOR_CALIBRATION[] = "catbot_msgs/set_motor_calibration";

  class set_motor_calibrationRequest : public ros::Msg
  {
    public:
      typedef int32_t _servo_index_type;
      _servo_index_type servo_index;
      typedef bool _setCalibration_servo_type;
      _setCalibration_servo_type setCalibration_servo;
      typedef float _min_calAngle_type;
      _min_calAngle_type min_calAngle;
      typedef float _max_calAngle_type;
      _max_calAngle_type max_calAngle;
      typedef float _min_posServo_type;
      _min_posServo_type min_posServo;
      typedef float _max_posServo_type;
      _max_posServo_type max_posServo;
      typedef float _min_posAngle_type;
      _min_posAngle_type min_posAngle;
      typedef float _max_posAngle_type;
      _max_posAngle_type max_posAngle;
      typedef float _min_lim_angle_type;
      _min_lim_angle_type min_lim_angle;
      typedef float _max_lim_angle_type;
      _max_lim_angle_type max_lim_angle;

    set_motor_calibrationRequest():
      servo_index(0),
      setCalibration_servo(0),
      min_calAngle(0),
      max_calAngle(0),
      min_posServo(0),
      max_posServo(0),
      min_posAngle(0),
      max_posAngle(0),
      min_lim_angle(0),
      max_lim_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_servo_index;
      u_servo_index.real = this->servo_index;
      *(outbuffer + offset + 0) = (u_servo_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_index);
      union {
        bool real;
        uint8_t base;
      } u_setCalibration_servo;
      u_setCalibration_servo.real = this->setCalibration_servo;
      *(outbuffer + offset + 0) = (u_setCalibration_servo.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->setCalibration_servo);
      union {
        float real;
        uint32_t base;
      } u_min_calAngle;
      u_min_calAngle.real = this->min_calAngle;
      *(outbuffer + offset + 0) = (u_min_calAngle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_calAngle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_calAngle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_calAngle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_calAngle);
      union {
        float real;
        uint32_t base;
      } u_max_calAngle;
      u_max_calAngle.real = this->max_calAngle;
      *(outbuffer + offset + 0) = (u_max_calAngle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_calAngle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_calAngle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_calAngle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_calAngle);
      union {
        float real;
        uint32_t base;
      } u_min_posServo;
      u_min_posServo.real = this->min_posServo;
      *(outbuffer + offset + 0) = (u_min_posServo.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_posServo.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_posServo.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_posServo.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_posServo);
      union {
        float real;
        uint32_t base;
      } u_max_posServo;
      u_max_posServo.real = this->max_posServo;
      *(outbuffer + offset + 0) = (u_max_posServo.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_posServo.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_posServo.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_posServo.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_posServo);
      union {
        float real;
        uint32_t base;
      } u_min_posAngle;
      u_min_posAngle.real = this->min_posAngle;
      *(outbuffer + offset + 0) = (u_min_posAngle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_posAngle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_posAngle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_posAngle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_posAngle);
      union {
        float real;
        uint32_t base;
      } u_max_posAngle;
      u_max_posAngle.real = this->max_posAngle;
      *(outbuffer + offset + 0) = (u_max_posAngle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_posAngle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_posAngle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_posAngle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_posAngle);
      union {
        float real;
        uint32_t base;
      } u_min_lim_angle;
      u_min_lim_angle.real = this->min_lim_angle;
      *(outbuffer + offset + 0) = (u_min_lim_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_lim_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_lim_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_lim_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_lim_angle);
      union {
        float real;
        uint32_t base;
      } u_max_lim_angle;
      u_max_lim_angle.real = this->max_lim_angle;
      *(outbuffer + offset + 0) = (u_max_lim_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_lim_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_lim_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_lim_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_lim_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_servo_index;
      u_servo_index.base = 0;
      u_servo_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_index = u_servo_index.real;
      offset += sizeof(this->servo_index);
      union {
        bool real;
        uint8_t base;
      } u_setCalibration_servo;
      u_setCalibration_servo.base = 0;
      u_setCalibration_servo.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->setCalibration_servo = u_setCalibration_servo.real;
      offset += sizeof(this->setCalibration_servo);
      union {
        float real;
        uint32_t base;
      } u_min_calAngle;
      u_min_calAngle.base = 0;
      u_min_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_calAngle = u_min_calAngle.real;
      offset += sizeof(this->min_calAngle);
      union {
        float real;
        uint32_t base;
      } u_max_calAngle;
      u_max_calAngle.base = 0;
      u_max_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_calAngle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_calAngle = u_max_calAngle.real;
      offset += sizeof(this->max_calAngle);
      union {
        float real;
        uint32_t base;
      } u_min_posServo;
      u_min_posServo.base = 0;
      u_min_posServo.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_posServo.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_posServo.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_posServo.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_posServo = u_min_posServo.real;
      offset += sizeof(this->min_posServo);
      union {
        float real;
        uint32_t base;
      } u_max_posServo;
      u_max_posServo.base = 0;
      u_max_posServo.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_posServo.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_posServo.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_posServo.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_posServo = u_max_posServo.real;
      offset += sizeof(this->max_posServo);
      union {
        float real;
        uint32_t base;
      } u_min_posAngle;
      u_min_posAngle.base = 0;
      u_min_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_posAngle = u_min_posAngle.real;
      offset += sizeof(this->min_posAngle);
      union {
        float real;
        uint32_t base;
      } u_max_posAngle;
      u_max_posAngle.base = 0;
      u_max_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_posAngle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_posAngle = u_max_posAngle.real;
      offset += sizeof(this->max_posAngle);
      union {
        float real;
        uint32_t base;
      } u_min_lim_angle;
      u_min_lim_angle.base = 0;
      u_min_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_lim_angle = u_min_lim_angle.real;
      offset += sizeof(this->min_lim_angle);
      union {
        float real;
        uint32_t base;
      } u_max_lim_angle;
      u_max_lim_angle.base = 0;
      u_max_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_lim_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_lim_angle = u_max_lim_angle.real;
      offset += sizeof(this->max_lim_angle);
     return offset;
    }

    virtual const char * getType() override { return SET_MOTOR_CALIBRATION; };
    virtual const char * getMD5() override { return "0600e947f1e961d431f7d8962da5e976"; };

  };

  class set_motor_calibrationResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;
      typedef const char* _info_type;
      _info_type info;

    set_motor_calibrationResponse():
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

    virtual const char * getType() override { return SET_MOTOR_CALIBRATION; };
    virtual const char * getMD5() override { return "929b8c0d7b68510a3f501a60258c746e"; };

  };

  class set_motor_calibration {
    public:
    typedef set_motor_calibrationRequest Request;
    typedef set_motor_calibrationResponse Response;
  };

}
#endif
