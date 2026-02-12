#include "Arduino.h"
#include "configuration.h"

#ifndef FEET_CONTACT_DETECTOR
#define FEET_CONTACT_DETECTOR


class FeetContactDetector{
 public:
  FeetContactDetector(int analogPin);
  ~FeetContactDetector(){};

  bool getStance();
  int get_analog_val();

 private:
  int analogPin;
  
};

# endif
