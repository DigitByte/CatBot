//
// Created by andres on 11.10.20.
//

#ifndef SRC_KINEMATICS_H
#define SRC_KINEMATICS_H


#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <urdf/model.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include <math.h>

#define PI 3.1415926535897932384626433832795028


class Kinematics {
public:
    Kinematics(std::string urdf_path, std::string legs_format);
    ~Kinematics();

    void getLegDimentions();
    void getTransform(const Eigen::Vector3f &pos, const Eigen::Vector3f &rpy, Eigen::Matrix4f* T);
    void getTransform(const Eigen::Vector3f &pos, const Eigen::Matrix3f &rot_matrix, Eigen::Matrix4f* T);

    void getRotationMatrix(const Eigen::Vector3f &rpy, Eigen::Matrix3f* R);
    void applyTransform(const Eigen::Vector3f &v, const Eigen::Matrix4f &transform, Eigen::Vector3f *v_trans);
    bool IK_leg_fromRef(std::string leg, const Eigen::Vector3f &feet_pos, Eigen::Vector3f* angles);
    void FK_leg_fromRef(std::string leg, const Eigen::Vector3f &angles, Eigen::Vector3f* feet_pos);
    void setBodyPose( const Eigen::Vector3f &body_pos, const Eigen::Vector3f &body_rpy);
    void getLegRefPosition(std::string leg, Eigen::Vector3f* href);
    void getLegRefRelativePosition(std::string leg, Eigen::Vector3f* href);

    bool feetAnglesFromRef(std::string leg, const Eigen::Vector3f &feet_pos_from_ref, Eigen::Vector3f* angles);
    void feetPosFromRef(std::string leg, const Eigen::Vector3f &angles, Eigen::Vector3f* feet_pos_from_ref);
    void getFeetFromBody(std::string leg, const Eigen::Vector3f &feet_pos_from_ref, Eigen::Vector3f* feet_pos_from_body);
    void feetPosFromBase(std::string leg, const Eigen::Vector3f &angles, Eigen::Vector3f* feet_pos_from_base);
    void getRefVelFromBodyVel(std::string leg, const Eigen::Vector3f &body_vel, Eigen::Vector3f* ref_vel);


    void getJacobian(std::string leg, const Eigen::Vector3f &joint_angles, Eigen::Matrix3f* Jacobian);
    void getPseudoInvJacobian(std::string leg, const Eigen::Vector3f &joint_angles, Eigen::Matrix3f* InvJacobian);

    void getForceReaction(std::string leg, const Eigen::Vector3f &joint_angles, const Eigen::Vector3f &joint_torques, Eigen::Vector3f* force);
    void getTorques(std::string leg, const Eigen::Vector3f &joint_angles, const Eigen::Vector3f &force, Eigen::Vector3f* joint_torques);

private:
    double constrainAngle(double x);

private:
    std::string urdf_path;
    std::string legs_format;
    Eigen::Vector3f offset_ref;
    Eigen::Vector3f offset_hip1;
    Eigen::Vector3f offset_hip2;
    Eigen::Vector3f offset_knee;
    Eigen::Vector3f offset_feet;

    Eigen::Matrix4f T_body;
};


# endif //SRC_KINEMATICS_H
