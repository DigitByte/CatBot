//
// Created by andres on 14.10.20.
//

#ifndef SRC_POLYGON_SUPPORT_H
#define SRC_POLYGON_SUPPORT_H

#include <eigen3/Eigen/Core>
#include <cmath>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono>


class PredPolygonSupport {
public:
    PredPolygonSupport();
    ~PredPolygonSupport();

    void setLeg(std::string leg, const Eigen::Vector3f feet_pos, float weight_factor);
    void getDesiredBodyPos(Eigen::Vector3f *body_pos);
private:
    void getVirtualVertice(std::string leg, Eigen::Vector3f* pos_vertice);

private:
    Eigen::Vector3f pos_feet_fl;
    Eigen::Vector3f pos_feet_fr;
    Eigen::Vector3f pos_feet_bl;
    Eigen::Vector3f pos_feet_br;

    float weight_factor_fl;
    float weight_factor_fr;
    float weight_factor_bl;
    float weight_factor_br;


};


# endif //SRC_POLYGON_SUPPORT_H
