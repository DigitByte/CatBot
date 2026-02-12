#include "eye.h"

Eye::Eye(Adafruit_SH1106G* display){
  this->display = display;
}


void Eye::begin(){
  display->begin(0x3C, true);
  this->display->clearDisplay();
  this->display->setRotation(0);

}


void Eye::draw(float center_x, float center_y, float length_x, float length_y ){
  
  int center_x_ix = center_x * this->display->width();
  int center_y_ix = center_y * this->display->height();
  int length_x_ix = length_x * this->display->width();
  int length_y_ix = length_y * this->display->height();

  int ini_pos_x = center_x_ix - length_x_ix/2;
  int ini_pos_y = center_y_ix - length_y_ix/2;
  

  this->display->clearDisplay();
  this->display->fillRoundRect(ini_pos_x, ini_pos_y, length_x_ix, length_y_ix, this->display->height()/16, SH110X_INVERSE);

}

void Eye::update(){
  this->display->display();
}

void Eye::clear(){
  
}
