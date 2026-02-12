//
// Created by andres on 07.11.20.
//

#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <urdf/model.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "settings.h"
#include <math.h>
#include "kinematics.h"
#include <fstream>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>


#define STATE_DIM 18
#define MEAS_DIM  28 //16

class StateEstimation {
public:
    StateEstimation(Kinematics* kinematics);

    ~StateEstimation();

    void reset();

    void setInitialState(const Eigen::Vector4d &base_quaternion_, const Eigen::Vector3f &base_pos, float *jointPos);

    void setInput(const Eigen::Vector4d &base_quaternion_, const Eigen::Vector3f &acc_, bool *feet_stance, float deltat);


    void predictState();

    void getFeetFromBaseRotated(std::string leg, const Eigen::Vector3f joint_angles, Eigen::Matrix<float, 3, 1>* feet_pos_from_base_rotated);

    void setMeasurement(const Eigen::Vector3f gyro, float *jointPos, float *jointVel, float *feetHeight_);
    void setFeetVel(std::string leg, const Eigen::Vector3f foot_vel);

    void updateState();

    void getBasePosition(Eigen::Vector3f* base_pos);
    void getBaseVelocity(Eigen::Vector3f* base_vel);
    void getBaseAngularVelocity(Eigen::Vector3f* base_angular_vel);
    void getFeetPosition(std::string leg, Eigen::Vector3f* feet_pos);
    void getBaseVelocityFromBaseRef(Eigen::Vector3f* base_vel);
    void getBaseAngularVelocityFromBaseRef(Eigen::Vector3f* base_ang_vel);
    void getBaseRotationMatrix(Eigen::Matrix3f* base_rot_matrix);
    void getBaseAccelerationFromBaseRef(Eigen::Vector3f* base_acc);
    float getMeanFeetHeight();
    void getCoMPosition(Eigen::Vector3f* com_pos);

private:
    void getFeetPositionCovariance(bool feet_stance, Eigen::Matrix3d* Qfeet);
    void getRelativeFeetVelocityCovariance(bool feet_stance, Eigen::Matrix3d* Qfeet_vel);
    void getRelativeFeetPositionCovariance(bool feet_stance, Eigen::Matrix3d* Qfeet_pos);

    double getFeetHeightCovariance(bool feet_stance);


private:
    Kinematics*         kinematics;


    double dt;
    bool *feet_stance;
    float* joint_angles;
    Eigen::Vector4d base_quaternion;
    Eigen::Vector3d angular_vel;
    Eigen::Matrix<double, STATE_DIM, 1> x;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;

    Eigen::Matrix3d base_rotation_matrix;
    Eigen::Matrix<double, 3, 1> acc;

    Eigen::Matrix<double, MEAS_DIM, 1> z;
    Eigen::Matrix<double, MEAS_DIM, STATE_DIM> H;
    Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> R;

    Eigen::Matrix3d O;
    Eigen::Matrix3d I;

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> I_state_dim;

    Eigen::Vector3f foot_vel_fl;
    Eigen::Vector3f foot_vel_fr;
    Eigen::Vector3f foot_vel_bl;
    Eigen::Vector3f foot_vel_br;

};

# endif