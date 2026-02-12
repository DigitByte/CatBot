//
// Created by andres on 22.10.20.
//

#ifndef SRC_CONTACT_DETECTION_H
#define SRC_CONTACT_DETECTION_H

#include <eigen3/Eigen/Core>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <cmath>
#include "settings.h"

class ContactDetection {
public:
    ContactDetection();
    ~ContactDetection();

    bool isFeetStanding(Eigen::Vector3f* force);

private:
    float force_threshold;
};


# endif //SRC_CONTACT_DETECTION_H
