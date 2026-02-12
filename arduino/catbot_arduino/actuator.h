#include "Arduino.h"
#include <Servo.h>
#include "configuration.h"
#include <ros.h>



class Actuator{
 public:
  Actuator(ros::NodeHandle *nh, int id, String name, int pwmPin);
  ~Actuator();

  void connect()   {this->servo->attach(this->pwmPin);};
  void disconnect();

  void setServoCalibration(float min_calAngle,  float max_calAngle,
                           float min_posServo,  float max_posServo,
                           float min_posAngle,  float max_posAngle);
                      
                      
  void setServoCalibration(bool set) {this->servo_calibrated = set;};
  bool isServoCalibrated(){return this->servo_calibrated;};

  void setLimPositions(float min_val, float max_val);
                      
  void setPosition(double pos);

  void run(double dt);

  float getTorque();
  float getPosition();
  float getVelocity();
  
  String getName(){return this->name;};

 private:
  void getPosAndVel(double dt);
  void setRawServoPos();
  

 private:

  ros::NodeHandle *nh;
 
  int    id;
  String name;
  int    pwmPin;

  float  setPos;
  float  curPos;
  float  prevPos;
  float  curVel;
  float  prevVel;
  
  Servo* servo;


  float min_calAngle;
  float max_calAngle;
  float min_posServo;
  float max_posServo;
  float min_posAngle;
  float max_posAngle;
  
  bool servo_calibrated;
  
  float lim_min_pos;
  float lim_max_pos;
  
};
