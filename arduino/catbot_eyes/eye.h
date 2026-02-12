#ifndef EYE_H
#define EYE_H

#include <Adafruit_SH110X.h>


class Eye{
public:
  Eye(Adafruit_SH1106G* display);
  ~Eye(){};

  
  void begin();

  void draw(float center_x, float center_y, float length_x, float length_y );
  void update();
  void clear();
  

private:
Adafruit_SH1106G* display;
  
};


# endif // EYE_H
