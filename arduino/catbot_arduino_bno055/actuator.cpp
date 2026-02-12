#include "actuator.h"



float mapVariable(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min)* (out_max - out_min)/ (in_max - in_min) + out_min;
}






Actuator::Actuator(ros::NodeHandle *nh, int id, String name, int pwmPin){
  this->nh = nh;
  this->id        = id;
  this->name      = name;
  this->pwmPin    = pwmPin;


  this->setPos        = 1500;
  this->curPos        = 1500;
  this->prevPos       = 0;

  
  this->curVel      = 0;
  this->prevVel     = 0;


  this->min_calAngle = 0;
  this->max_calAngle = 0;
  this->min_posServo = 0;
  this->max_posServo = 0;
  this->min_posAngle = 0;
  this->max_posAngle = 0;
  
  

  this->lim_min_pos = 0;
  this->lim_max_pos = 0;

  this->servo = new Servo;
  // this->servo->attach(this->pwmPin);

  this->servo_calibrated       = false;

}

Actuator::~Actuator(){
  
}

void Actuator::disconnect(){
  this->servo->detach();
}


void Actuator::setLimPositions(float min_val, float max_val){
  this->lim_min_pos = min_val;
  this->lim_max_pos = max_val;
}

void Actuator::setServoCalibration(float min_calAngle,  float max_calAngle,
                                      float min_posServo,  float max_posServo,
                                      float min_posAngle,  float max_posAngle){

  this->min_calAngle  = min_calAngle;
  this->max_calAngle  = max_calAngle;
  
  this->min_posServo  = min_posServo;
  this->max_posServo  = max_posServo;
  
  this->min_posAngle  = min_posAngle;
  this->max_posAngle  = max_posAngle;

  this->setPos = this->min_calAngle + (this->max_calAngle - this->min_calAngle)/2;

  this->servo_calibrated = true;
  
}




float Actuator::getPosition(){
  return this->curPos;
}

float Actuator::getVelocity(){
  return this->curVel;
}

float Actuator::getTorque(){
    return 0;

}


void Actuator::setPosition(double pos){
  this->setPos = pos;
}


void Actuator::run(double dt){
  this->setRawServoPos();
  this->getPosAndVel(dt);
}












void Actuator::setRawServoPos(){ 
  float val;
  if (this->servo_calibrated){
    
    if (this->setPos > this->lim_max_pos)      this->setPos = this->lim_max_pos;
    else if (this->setPos < this->lim_min_pos) this->setPos = this->lim_min_pos;
    
    val = mapVariable(this->setPos,
                               this->min_calAngle, this->max_calAngle,
                               this->min_posServo, this->max_posServo);                
  }
  else{
    val = this->setPos;
  }
  this->servo->writeMicroseconds(val);
}




void Actuator::getPosAndVel(double dt){
  this->curPos     = this->setPos;;
  this->curVel     = (1-VELOCITY_FILTER)*this->prevVel + VELOCITY_FILTER*(this->curPos - this->prevPos)/dt;
  this->prevVel    = this->curVel;
  this->prevPos    = this->curPos;


}
