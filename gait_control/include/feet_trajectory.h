//
// Created by andres on 13.10.20.
//

#ifndef SRC_FEET_TRAJECTORY_H
#define SRC_FEET_TRAJECTORY_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <cmath>
#include "kinematics.h"
#include "settings.h"
#include "ros/ros.h"

enum FEET_STATE{
    STANCE,
    SWING
};

enum STOP_REQUEST_STATE{
    NON_STOP_REQUEST,
    REQUESTED_TO_STOP,
    MADE_LAST_STEP
};


class FeetTrajectory {
public:
    FeetTrajectory(std::string name);
    ~FeetTrajectory();

    void setKinematics(Kinematics* kinematics);

    void getFeetState(double main_phase, bool foot_contact);
    void makeStep(bool foot_contact,
                  const Eigen::Vector3f &leg_des_dir,
                  const Eigen::Vector3f &des_base_vel,
                  const Eigen::Vector3f &base_vel,
                  double main_phase,
                  float base_height,
                  Eigen::Vector3f *feet_pos);

    void setPeriodAndSwing(double period, double prop_time_swing);

    void setTrajectoryParams(double phase_offset,
                             float height,
                             float feet_offset_x,
                             float feet_offset_y,
                             float base_offset_x,
                             float base_offset_y,
                             float capure_point_weight);

    void setFootContactParams(float min_prop_swing_time_before_contact_detection,
                              float dt_contact_detection_seconds);

    void setWalking(bool walking);
    void setWalkingState(bool walking){this->walking = walking;};
    void getFeetOffset(Eigen::Vector3f *feet_pos);

    float getWeightingFactor(float sigma1, float sigma2, float sigma1_, float sigma2_, float offset);
    bool getStance();
    void useIntegratedPos(bool set){this->use_integrated_pos = set;};

    // This function is used by the predictive model controller
    bool predictFeetState(double current_main_phase,
                          double dt,
                          const Eigen::Vector3d &ref_base_pos,
                          const Eigen::Vector3d &ref_base_vel,
                          const Eigen::Vector3d &ref_base_ang,
                          const Eigen::Vector3d &ref_base_ang_vel,
                          const Eigen::Vector3d &current_foot_pos,
                          Eigen::Vector3d* predicted_foot_pos);


    void updateFeePos(){this->current_pos = this->current_pos_tmp;};

    void getVelocityFromBase(Eigen::Vector3f* vel ){ vel->operator=( this-> foot_vel );}

public:
    std::string leg_name;
    double phase_offset;
    double leg_phase;
    double feet_state_phase;
    FEET_STATE feet_state;
    FEET_STATE feet_prev_state;

    STOP_REQUEST_STATE stop_request_state;
    bool walking;

    Eigen::Vector3f initial_pos;
    Eigen::Vector3f current_pos;
    Eigen::Vector3f current_pos_tmp;
    Eigen::Vector3f middle_pos;
    Eigen::Vector3f final_pos;
    Eigen::Vector3f integrated_feet_pos;
    Eigen::Vector3f foot_vel;

    bool set_initial_pos;
    bool use_integrated_pos;
    bool early_contact;

    double prop_time_swing;

    double period;
    float height;
    ros::Time timer;
    ros::Time contactTimer;
    bool prev_contact;

    float min_prop_swing_time_before_contact_detection;
    float dt_contact_detection_seconds;

    Eigen::Vector3f feet_pos_offset;
    Eigen::Vector3f base_pos_offset;

    Eigen::Vector3f prev_feet_vel;

    Kinematics*              kinematics;
    float max_leg_displacement;
    double capure_point_weight;

};


# endif //SRC_FEET_TRAJECTORY_H
