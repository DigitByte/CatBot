//
// Created by andres on 20.03.21.
//

#ifndef SRC_PREDICTIVE_MODEL_CONTROL_H
#define SRC_PREDICTIVE_MODEL_CONTROL_H


#include "eigen-qp.h"
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <urdf/model.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include "feet_trajectory.h"
#include "kinematics.h"
#include "settings.h"
#include <math.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include "QuadProgPP.h"
#include <qpOASES.hpp>

#define DYN_STATE_DIM 15


typedef Eigen::Matrix<double, DYN_STATE_DIM, DYN_STATE_DIM> Matrix13d;
typedef Eigen::Matrix<double, DYN_STATE_DIM, 12> Matrix13x12d;
typedef Eigen::Matrix<double, DYN_STATE_DIM, 1> Vector13d;
// typedef Eigen::Matrix<float, DYN_STATE_DIM * HORIZON, 1> Vector13kf;

USING_NAMESPACE_QPOASES

class PredictiveModeController {
public:
    PredictiveModeController(FeetTrajectory* fl_trajectory, FeetTrajectory* fr_trajectory, FeetTrajectory* bl_trajectory, FeetTrajectory* br_trajectory, Kinematics* kinematics);
    ~PredictiveModeController();

    void setHorizon_and_dtime(int horizon, double dt);
    void setBaseCoMOffset(double base_offset_x, double base_offset_y);
    void setTerrainPlane(std::string leg, double mu, const Eigen::Vector3d &normal, const Eigen::Vector3d &tangent1, const Eigen::Vector3d &tangent2, double yaw);
    void setFeetForceLimits(double min_force, double max_force);
    void setMaxFeetVelocity(double max_feet_vel);
    void setFeetPos(std::string leg, const Eigen::Vector3d &feet_pos);
    void setWeightsController(const Eigen::Vector3d &base_pos_weight, const Eigen::Vector3d &base_ang_weight, const Eigen::Vector3d &base_vel_weight, const Eigen::Vector3d &base_ang_vel_weight,  double w_forces);

    void setRobotState(const Eigen::Vector3d base_pos, const Eigen::Vector3d base_vel, const Eigen::Vector3d base_ang, const Eigen::Vector3d base_ang_vel);
    void setRobotControl(const Eigen::Vector3d base_vel, const Eigen::Vector3d des_base_ang_vel);


    void computeForces(double current_main_phase);
    void computeForcesQPoases(double current_main_phase);


    void getForces(Eigen::Vector3d* force_fl, Eigen::Vector3d* force_fr, Eigen::Vector3d* force_bl, Eigen::Vector3d* force_br);
    void getBaseVelWeight(Eigen::Vector3d* weight){weight->operator=( this->base_vel_weight );};

private:
void getRz(double yaw, Eigen::Matrix3d* Rz);
void getCrossProductMatrix(Eigen::Vector3d r, Eigen::Matrix3d* C);

void getAmatrix(double yaw, const Eigen::Matrix3d &InertiaInv,
                const Eigen::Vector3d &pos_fl,
                const Eigen::Vector3d &pos_fr,
                const Eigen::Vector3d &pos_bl,
                const Eigen::Vector3d &pos_br,
                bool*  stance_feets,
                Matrix13d* A);

void getBmatrix(double yaw,
                const Eigen::Matrix3d &InertiaInv,
                const Eigen::Vector3d &pos_fl,
                const Eigen::Vector3d &pos_fr,
                const Eigen::Vector3d &pos_bl,
                const Eigen::Vector3d &pos_br,
                int num_stance_feet,
                Matrix13x12d* B);

void getAdynMatrix(double yaw,  double dt, const Eigen::Matrix3d &InertiaInv,
                   const Eigen::Vector3d &pos_fl,
                   const Eigen::Vector3d &pos_fr,
                   const Eigen::Vector3d &pos_bl,
                   const Eigen::Vector3d &pos_br,
                   bool*  stance_feets,
                   Matrix13d* Adyn);

void getBdynMatrix(double yaw, double dt,
                   const Eigen::Matrix3d &InertiaInv,
                   const Eigen::Vector3d &pos_fl,
                   const Eigen::Vector3d &pos_fr,
                   const Eigen::Vector3d &pos_bl,
                   const Eigen::Vector3d &pos_br,
                   int num_stance_feet,
                   Matrix13x12d* Bdyn);

void getAdynAndBdynMatrices(double dt, const Matrix13d &A, const Matrix13x12d &B, Matrix13d *Adyn, Matrix13x12d* Bdyn);


private:

    int   HORIZON;
    double DELTAT;


    Eigen::MatrixXd state_ref;

    // Vector13kf state_ref;
    Vector13d  initial_state;

    Eigen::Vector3d base_pos;
    Eigen::Vector3d base_ang;
    Eigen::Vector3d base_vel;
    Eigen::Vector3d base_ang_vel;

    Eigen::Vector3d feet_pos_fl;
    Eigen::Vector3d feet_pos_fr;
    Eigen::Vector3d feet_pos_bl;
    Eigen::Vector3d feet_pos_br;

    Eigen::Vector3d base_vel_weight;


    Eigen::Matrix3d I3;
    Eigen::Matrix3d O3;
    Eigen::Vector3d base_CoM_offset;
    Eigen::Matrix<double, 1, 3> O3H;
    Eigen::Matrix<double, 3, 1> O3V;

    Eigen::Matrix3d Inertia;

    FeetTrajectory* fl_trajectory;
    FeetTrajectory* fr_trajectory;
    FeetTrajectory* bl_trajectory;
    FeetTrajectory* br_trajectory;

    Kinematics*              kinematics;


    Eigen::Matrix<double,5,3> terrain_fl;
    Eigen::Matrix<double,5,3> terrain_fr;
    Eigen::Matrix<double,5,3> terrain_bl;
    Eigen::Matrix<double,5,3> terrain_br;

    Eigen::Matrix<double, 5,1> d_min;
    Eigen::Matrix<double, 5,1> d_max;

    double max_feet_vel;

    Matrix13d w_dyn_model;
    double w_forces;

    bool stance_fl;
    bool stance_fr;
    bool stance_bl;
    bool stance_br;
    Eigen::VectorXd curr_forces;


    SQProblem* solve_qp;
    bool init_qp;
};



# endif //SRC_PREDICTIVE_MODEL_CONTROL_H
