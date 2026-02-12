//
// Created by andres on 07.11.20.
//

#ifndef SRC_TORQUE_CONTROLLER_H
#define SRC_TORQUE_CONTROLLER_H

#include "eigen-qp.h"
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <urdf/model.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Dense>

#include "settings.h"
#include "kinematics.h"
#include <math.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include "QuadProgPP.h"


class torqueController {
public:
    torqueController(Kinematics* kinematics);
    ~torqueController();

    void setLegState(std::string leg, const Eigen::Vector3f &feet_pos, bool stance_state);
    void setTerrainPlane(std::string leg, float mu, const Eigen::Vector3f &normal, const Eigen::Vector3f &tagent1, const Eigen::Vector3f &tangent2);
    void setFeetForceLimits(float min_force, float max_force);
    void setProportionalConstController(Eigen::Vector3f k_pos, Eigen::Vector3f k_vel, Eigen::Vector3f k_rot, Eigen::Vector3f k_ang_vel);
    void setWeightsController(float w_dyn_model, float w_forces, float w_prev_forces);
    void setBaseCoMOffset(float base_offset_x, float base_offset_y);

    void setRobotState  (const Eigen::Vector3f &base_pos,     const Eigen::Vector3f &base_vel,     const Eigen::Matrix3f &base_rot, const Eigen::Vector3f &base_ang_vel);
    void setRobotControl(const Eigen::Vector3f &des_base_pos, const Eigen::Vector3f &des_base_vel, const Eigen::Matrix3f &des_base_rot, const Eigen::Vector3f &des_base_ang_vel);

    void computeForces(double dt);

    void getForces(Eigen::Vector3f* force_fl, Eigen::Vector3f* force_fr, Eigen::Vector3f* force_bl, Eigen::Vector3f* force_br);



    void getKvel(Eigen::Vector3f * k_vel_vector);

private:
    void getVectorialMulMatrix(const Eigen::Vector3f &v, Eigen::Matrix3f* m);
    int countStanceFeets();

private:

    Kinematics* kinematics;
    Eigen::Matrix3f Inertia;

    Eigen::Vector3f base_pos_offset;


    Eigen::Vector3f base_pos;
    Eigen::Vector3f base_vel;
    Eigen::Matrix3f base_rot;
    Eigen::Vector3f base_ang_vel;

    Eigen::Vector3f des_base_pos;
    Eigen::Vector3f des_base_vel;
    Eigen::Matrix3f des_base_rot;
    Eigen::Vector3f des_base_ang_vel;


    Eigen::Vector3f feet_pos_fl;
    Eigen::Vector3f feet_pos_fr;
    Eigen::Vector3f feet_pos_bl;
    Eigen::Vector3f feet_pos_br;

    Eigen::Matrix<float,5,3> terrain_fl;
    Eigen::Matrix<float,5,3> terrain_fr;
    Eigen::Matrix<float,5,3> terrain_bl;
    Eigen::Matrix<float,5,3> terrain_br;

    Eigen::Matrix<float, 5,1> d_min;
    Eigen::Matrix<float, 5,1> d_max;



    bool stance_fl;
    bool stance_fr;
    bool stance_bl;
    bool stance_br;

    int num_stand_feets;
    Eigen::VectorXf prev_forces;
    Eigen::VectorXf curr_forces;

    Eigen::Matrix3f k_pos;
    Eigen::Matrix3f k_vel;
    Eigen::Matrix3f k_rot;
    Eigen::Matrix3f k_ang_vel;

    float w_dyn_model;
    float w_forces;
    float w_prev_forces;
};


# endif //SRC_TORQUE_CONTROLLER_H
