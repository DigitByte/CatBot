//
// Created by andres on 22.10.20.
//

#include "../include/contact_detection.h"


ContactDetection::ContactDetection(){
    this->force_threshold = FORCE_CONTAT_THRESHOLD;
}

ContactDetection::~ContactDetection(){

}


bool ContactDetection::isFeetStanding(Eigen::Vector3f* force){
    /* So far a simple method, but it could be improved */
    if(force->operator()(2) > this->force_threshold ){
        return true;
    }
    return false;
}
