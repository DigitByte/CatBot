#include "face.h"




Face::Face(Adafruit_SH1106G* displayL, Adafruit_SH1106G* displayR){
  this->eyeL = new Eye(displayL);
  this->eyeR = new Eye(displayR);

  center_x = 0.5;
  center_y = 0.5;
  length_x = 0.4;
  length_y = 0.5;
  
}

void Face::setStyle(faceStyles style){
  switch(style){
      case LONG_EYES  : length_x = 0.4; length_y = 0.5;
      case SQUARE_EYES: length_x = 0.4; length_y = 0.4;
      case WIDE_EYES  : length_x = 0.4; length_y = 0.3;
  }
}



void Face::begin(){
  this->eyeL->begin();
  this->eyeR->begin();
}

void Face::drawEyes(float center_x, float center_y, float length_x, float length_y ){
  this->eyeL->draw(center_x, center_y, length_x, length_y);

  if (INVERT == 1){
      this->eyeR->draw(1-center_x, 1-center_y, length_x, length_y);
  }
  else{
    this->eyeR->draw(center_x, center_y, length_x, length_y);
  }
  
}

void Face::update(){
  this->eyeL->update();
  this->eyeR->update();
}


void Face::move(float pos_x, float pos_y, double period, int num_steps){
  float dx = (pos_x - this->center_x)/num_steps;
  float dy = (pos_y - this->center_y)/num_steps;

  double dt = (period/num_steps)*1000; // to ms


  for(int t=0; t<num_steps; t++){
    this->center_x = this->center_x + dx;
    this->center_y = this->center_y + dy;   
    Serial.println(this->center_x);
    this->drawEyes(this->center_x, this->center_y, this->length_x, this->length_y);
    this->update();
    delay(dt);
  }

  
}

void Face::blink(double period, int num_steps){
  double dt = (period/num_steps)*1000; // to ms

  float dl = 2*this->length_x/(num_steps);
  
  for(int t=0; t<num_steps/2; t++){
    this->length_x = this->length_x - dl;
    this->drawEyes(this->center_x, this->center_y, this->length_x, this->length_y);
    this->update();
    delay(dt);
  }
  
  for(int t=0; t<num_steps/2; t++){
    this->length_x = this->length_x + dl;
    this->drawEyes(this->center_x, this->center_y, this->length_x, this->length_y);
    this->update();
    delay(dt);

  }
  
}
