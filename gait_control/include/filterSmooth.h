#ifndef FILTERSMOOTH_H__
#define FILTERSMOOTH_H__

#include "geometry_msgs/TwistStamped.h"


class FilterSmooth {
 public:
    FilterSmooth();
    ~FilterSmooth(){};

    void setFactor(float factor){this->factor = factor;};
    void update(geometry_msgs::TwistStamped *input);
    float get(geometry_msgs::TwistStamped *output);
    geometry_msgs::TwistStamped get();

 private:
    geometry_msgs::TwistStamped cmd_filtered;
    float factor;
};



# endif