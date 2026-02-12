#include "FeetContactDetector.h"


FeetContactDetector::FeetContactDetector(int analogPin){
  this->analogPin = analogPin;
}


int FeetContactDetector::get_analog_val(){
    int val = analogRead(this->analogPin);
    return val;

}

bool FeetContactDetector::getStance(){
  
  int val = this->get_analog_val();
  if (val >= CONTACT_DETECTOR_ANALOG_THRESHOLD){
    return false;
  }

  return true;
}
