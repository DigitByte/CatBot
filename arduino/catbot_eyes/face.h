#ifndef FACE_H
#define FACE_H

#include "eye.h"
#include <Adafruit_SH110X.h>


#define INVERT 1

enum faceStyles{
  LONG_EYES,
  SQUARE_EYES,
  WIDE_EYES,
};


class Face{
public:
  Face(Adafruit_SH1106G* displayL, Adafruit_SH1106G* displayR);
  ~Face(){};

  
  void begin();
  void setStyle(faceStyles style);

  void drawEyes(float center_x, float center_y, float length_x, float length_y );
  void update();

  void blink(double period, int num_steps);
  void move(float pos_x, float pos_y, double period, int num_steps);

private:
  
  Eye* eyeL;
  Eye* eyeR;

  float center_x;
  float center_y;
  float length_x;
  float length_y;
  
};


# endif // FACE_H
