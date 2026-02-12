//
// Created by andres on 07.11.20.
//


#include "../include/state_estimation.h"


void crossProduct(const Eigen::Matrix<double, 3, 1> &v1, const Eigen::Matrix<double, 3, 1> &v2, Eigen::Matrix<double, 3, 1>* result){
    Eigen::Matrix<double, 3, 1> r;

    r << v1(1,0)*v2(2,0)- v1(2,0)*v2(1,0),   -v1(0,0)*v2(2,0) + v1(2,0)*v2(0,0),   v1(0,0)*v2(1,0)- v1(1,0)*v2(0,0);
    result->operator=( r );
}

StateEstimation::StateEstimation(Kinematics* kinematics){
    this->kinematics = kinematics;
    this->reset();


}

StateEstimation::~StateEstimation(){

}

void StateEstimation::reset() {
    this->x.setZero();
    this->dt = 0;
    this->P.fill(0.1);


    bool feet_stance[4] = {0,0,0,0};
    float joint_pos[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    this->feet_stance = feet_stance;
    this->joint_angles = joint_pos;

    this->O.setZero();

    this->I << 1,0,0,
               0,1,0,
               0,0,1;
    this->base_quaternion << 1,0,0,0;
    this->angular_vel     << 0,0,0;
    this->acc             << 0,0,0;

    this->base_rotation_matrix = this->I;

    Eigen::Matrix<double, 1, 3> vz, vO;
    vz << 0,0,-1;
    vO << 0,0, 0;


    this->foot_vel_fl.setZero();
    this->foot_vel_fr.setZero();
    this->foot_vel_bl.setZero();
    this->foot_vel_br.setZero();


    Eigen::Matrix<double, 3, STATE_DIM> H0, H1, H2, H3, H4, H5, H6, H7;
    Eigen::Matrix<double, 1, STATE_DIM> H8, H9, H10, H11;

    H0  <<  I,  O, -I,  O,  O,  O;
    H1  <<  I,  O,  O, -I,  O,  O;
    H2  <<  I,  O,  O,  O, -I,  O;
    H3  <<  I,  O,  O,  O,  O, -I;
    H4  <<  O, -I,  O,  O,  O,  O;
    H5  <<  O, -I,  O,  O,  O,  O;
    H6  <<  O, -I,  O,  O,  O,  O;
    H7  <<  O, -I,  O,  O,  O,  O;
    H8  << vO, vO, vz, vO, vO, vO;
    H9  << vO, vO, vO, vz, vO, vO;
    H10 << vO, vO, vO, vO, vz, vO;
    H11 << vO, vO, vO, vO, vO, vz;



    this->H << H0, H1, H2, H3, H4, H5, H6, H7, H8, H9, H10, H11;
    // this->H << H0, H1, H2, H3, H8, H9, H10, H11;


    Eigen::Matrix<double, 3, STATE_DIM> Idim_0, Idim_1, Idim_2, Idim_3, Idim_4, Idim_5;
    Idim_0 << I, O, O, O, O, O;
    Idim_1 << O, I, O, O, O, O;
    Idim_2 << O, O, I, O, O, O;
    Idim_3 << O, O, O, I, O, O;
    Idim_4 << O, O, O, O, I, O;
    Idim_5 << O, O, O, O, O, I;

    this->I_state_dim << Idim_0, Idim_1, Idim_2, Idim_3, Idim_4, Idim_5;


}

void StateEstimation::setInitialState(const Eigen::Vector4d &base_quaternion_, const Eigen::Vector3f &base_pos, float *jointPos) {
    this->base_quaternion = base_quaternion_;

    double s_ = this->base_quaternion.w();
    double x_ = this->base_quaternion.x();
    double y_ = this->base_quaternion.y();
    double z_ = this->base_quaternion.z();


    this->base_rotation_matrix << 1 - 2*y_*y_ - 2*z_*z_,  2*x_*y_ - 2*s_*z_,       2*x_*z_ + 2*s_*y_,
                                  2*x_*y_ + 2*s_*z_,      1 - 2*x_*x_ - 2*z_*z_,   2*y_*z_ - 2*s_*x_,
                                  2*x_*z_ - 2*s_*y_,      2*y_*z_ + 2*s_*x_,       1 - 2*x_*x_ - 2*y_*y_;

    this->joint_angles = jointPos;


    Eigen::Vector3f angles_fl, angles_fr, angles_bl, angles_br;
    angles_fl << jointPos[0], jointPos[1], jointPos[2];
    angles_fr << jointPos[3], jointPos[4], jointPos[5];
    angles_bl << jointPos[6], jointPos[7], jointPos[8];
    angles_br << jointPos[9], jointPos[10], jointPos[11];

    Eigen::Vector3f feet_rel_pos_fl, feet_rel_pos_fr, feet_rel_pos_bl, feet_rel_pos_br;
    this->getFeetFromBaseRotated("fl", angles_fl, &feet_rel_pos_fl);
    this->getFeetFromBaseRotated("fr", angles_fr, &feet_rel_pos_fr);
    this->getFeetFromBaseRotated("bl", angles_bl, &feet_rel_pos_bl);
    this->getFeetFromBaseRotated("br", angles_br, &feet_rel_pos_br);

    Eigen::Vector3f pos_feet_fl, pos_feet_fr, pos_feet_bl, pos_feet_br;
    pos_feet_fl = base_pos + feet_rel_pos_fl;
    pos_feet_fr = base_pos + feet_rel_pos_fr;
    pos_feet_bl = base_pos + feet_rel_pos_bl;
    pos_feet_br = base_pos + feet_rel_pos_br;

    // std::cout << "ang fl" << ": " <<  angles_fl.x() << " " << angles_fl.y() << " " << angles_fl.z() <<  std::endl;
    // std::cout << "ang fr" << ": " <<  angles_fr.x() << " " << angles_fr.y() << " " << angles_fr.z() <<  std::endl;
    // std::cout << "ang bl" << ": " <<  angles_bl.x() << " " << angles_bl.y() << " " << angles_bl.z() <<  std::endl;
    // std::cout << "ang br" << ": " <<  angles_br.x() << " " << angles_br.y() << " " << angles_br.z() <<  std::endl;


    // std::cout << "base" << ": " <<  base_pos.x() << " " << base_pos.y() << " " << base_pos.z() <<  std::endl;

    // std::cout << "fl" << ": " <<  pos_feet_fl.x() << " " << pos_feet_fl.y() << " " << pos_feet_fl.z() <<  std::endl;
    // std::cout << "fr" << ": " <<  pos_feet_fr.x() << " " << pos_feet_fr.y() << " " << pos_feet_fr.z() <<  std::endl;
    // std::cout << "bl" << ": " <<  pos_feet_bl.x() << " " << pos_feet_bl.y() << " " << pos_feet_bl.z() <<  std::endl;
    // std::cout << "br" << ": " <<  pos_feet_br.x() << " " << pos_feet_br.y() << " " << pos_feet_br.z() <<  std::endl;



    Eigen::Vector3d base_vel;
    base_vel << 0,0,0;


    this->x << base_pos.cast<double>()  ,
               base_vel   ,
               pos_feet_fl.cast<double>(),
               pos_feet_fr.cast<double>(),
               pos_feet_bl.cast<double>(),
               pos_feet_br.cast<double>();

}

void StateEstimation::getFeetPositionCovariance(bool feet_stance, Eigen::Matrix3d* Qfeet){
    double w = pow(STD_FEET_SWING, 2);
    if (feet_stance){
        w = pow(STD_FEET_STANCE, 2);
    }
    Eigen::Matrix3d Q_;
    Q_ << w, 0, 0,
         0, w, 0,
         0, 0, w;

    Qfeet->operator=(Q_);
}


void StateEstimation::getRelativeFeetVelocityCovariance(bool feet_stance, Eigen::Matrix3d* Qfeet_vel){
    double w = pow(STD_FEET_VEL_REL_SWING, 2);
    if (feet_stance){
        w = pow(STD_FEET_VEL_REL_STANCE, 2);
    }
    Eigen::Matrix3d Q_;
    Q_ << w, 0, 0,
         0, w, 0,
         0, 0, w;

    Qfeet_vel->operator=(Q_);
}

void StateEstimation::getRelativeFeetPositionCovariance(bool feet_stance, Eigen::Matrix3d* Qfeet_pos){
    double w = pow(STD_FEET_POS_REL_SWING, 2);
    if (feet_stance){
        w = pow(STD_FEET_POS_REL_STANCE, 2);
    }
    Eigen::Matrix3d Q_;
    Q_ << w, 0, 0,
         0, w, 0,
         0, 0, w;

    Qfeet_pos->operator=(Q_);
}



double StateEstimation::getFeetHeightCovariance(bool feet_stance){
    double w = pow(STD_FEET_HEIGHT_SWING, 2);
    if (feet_stance){
        w = pow(STD_FEET_HEIGHT_STANCE, 2);
    }
    return w;
}



void StateEstimation::setInput(const Eigen::Vector4d &base_quaternion_, const Eigen::Vector3f &acc_, bool *feet_stance, float deltat) {

    this->base_quaternion = base_quaternion_;
    this->feet_stance = feet_stance;
    double s_ = this->base_quaternion.w();
    double x_ = this->base_quaternion.x();
    double y_ = this->base_quaternion.y();
    double z_ = this->base_quaternion.z();

    this->base_rotation_matrix << 1 - 2*y_*y_ - 2*z_*z_,  2*x_*y_ - 2*s_*z_,       2*x_*z_ + 2*s_*y_,
                                  2*x_*y_ + 2*s_*z_,      1 - 2*x_*x_ - 2*z_*z_,   2*y_*z_ - 2*s_*x_,
                                  2*x_*z_ - 2*s_*y_,      2*y_*z_ + 2*s_*x_,       1 - 2*x_*x_ - 2*y_*y_;


    Eigen::Matrix<double, 3, 1> acc_tmp;
    acc_tmp << acc_.x(), acc_.y(), acc_.z();

    this->acc = this->base_rotation_matrix * acc_tmp;
    this->dt = deltat;

    Eigen::Matrix3d Q_acc;
    Q_acc <<  pow(STD_LIN_ACC_X,2), 0, 0,
              0, pow(STD_LIN_ACC_Y,2), 0,
              0, 0, pow(STD_LIN_ACC_Z,2);
    Q_acc = this->base_rotation_matrix.transpose()*Q_acc * this->base_rotation_matrix;  //new

    Eigen::Matrix3d Qfl_tf, Qfr_tf, Qbl_tf, Qbr_tf;
    this->getFeetPositionCovariance(feet_stance[0], &Qfl_tf);
    this->getFeetPositionCovariance(feet_stance[1], &Qfr_tf);
    this->getFeetPositionCovariance(feet_stance[2], &Qbl_tf);
    this->getFeetPositionCovariance(feet_stance[3], &Qbr_tf);

    Eigen::Matrix3d Qfl, Qfr, Qbl, Qbr;
    Qfl = this->base_rotation_matrix.transpose() * Qfl_tf * this->base_rotation_matrix;
    Qfr = this->base_rotation_matrix.transpose() * Qfr_tf * this->base_rotation_matrix;
    Qbl = this->base_rotation_matrix.transpose() * Qbl_tf * this->base_rotation_matrix;
    Qbr = this->base_rotation_matrix.transpose() * Qbr_tf * this->base_rotation_matrix;

    Eigen::Matrix3d Qacc1, Qacc2, Qacc3;

    Qacc1  = dt*Q_acc;

    double dt2 = pow(dt,2)/2;
    Qacc2 << dt2*Q_acc;

    double dt3 = pow(dt,3)/3;
    Qacc3 << dt3*Q_acc;


    Eigen::Matrix<double, 3, STATE_DIM> Q0, Q1, Q2, Q3, Q4, Q5;

    Q0 << Qacc3, Qacc2, O, O, O, O;
    Q1 << Qacc2, Qacc1, O, O, O, O;
    Q2 << O, O, Qfl, O, O, O;
    Q3 << O, O, O, Qfr, O, O;
    Q4 << O, O, O, O, Qbl, O;
    Q5 << O, O, O, O, O, Qbr;

    this->Q << Q0, Q1, Q2, Q3, Q4, Q5;
}


void StateEstimation::predictState() {
    Eigen::Matrix3d Idt, Idt2;
    double dt2 = pow(dt,2)/2;

    Idt << dt, 0,  0,
           0 , dt, 0,
           0,  0,  dt;

    Idt2 << dt2, 0,     0,
            0,    dt2,  0,
            0,    0,    dt2;

    Eigen::Matrix<double, 3, STATE_DIM> A0, A1, A2, A3, A4, A5;
    A0 << I, Idt, O, O, O, O;
    A1 << O, I, O, O, O, O;
    A2 << O, O, I, O, O, O;
    A3 << O, O, O, I, O, O;
    A4 << O, O, O, O, I, O;
    A5 << O, O, O, O, O, I;

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> A;
    A << A0, A1, A2, A3, A4, A5;

    Eigen::Matrix<double, STATE_DIM, 3> B;
    B << Idt2,
         Idt,
         O,
         O,
         O,
         O;



    this->x = A * this->x + B * this->acc;
    this->P = A* this->P * A.transpose() + this->Q;


}


void StateEstimation::getFeetFromBaseRotated(std::string leg, const Eigen::Vector3f joint_angles, Eigen::Matrix<float, 3, 1>* feet_pos_from_base_rotated){
    Eigen::Vector3f feet_from_base;
    this->kinematics->feetPosFromBase(leg, joint_angles, &feet_from_base);

    Eigen::Matrix<float, 3, 1> feet_from_base_col;

    feet_from_base_col << feet_from_base.x(), feet_from_base.y(), feet_from_base.z();

    feet_pos_from_base_rotated->operator=( this->base_rotation_matrix.cast<float>() * feet_from_base_col );

}

void StateEstimation::setFeetVel(std::string leg, const Eigen::Vector3f foot_vel){

    if (leg == "fl") this->foot_vel_fl = foot_vel;
    if (leg == "fr") this->foot_vel_fr = foot_vel;
    if (leg == "bl") this->foot_vel_bl = foot_vel;
    if (leg == "br") this->foot_vel_br = foot_vel;

}


void StateEstimation::setMeasurement(const Eigen::Vector3f gyro, float *jointPos, float *jointVel, float *feetHeight_) {
    this->angular_vel = this->base_rotation_matrix * (gyro.cast<double>());

    Eigen::Vector3f angles_fl, angles_fr, angles_bl, angles_br;
    angles_fl << jointPos[0], jointPos[1], jointPos[2];
    angles_fr << jointPos[3], jointPos[4], jointPos[5];
    angles_bl << jointPos[6], jointPos[7], jointPos[8];
    angles_br << jointPos[9], jointPos[10], jointPos[11];

    Eigen::Matrix<float, 3, 1> feet_rel_pos_fl, feet_rel_pos_fr, feet_rel_pos_bl, feet_rel_pos_br;
    this->getFeetFromBaseRotated("fl", angles_fl, &feet_rel_pos_fl);
    this->getFeetFromBaseRotated("fr", angles_fr, &feet_rel_pos_fr);
    this->getFeetFromBaseRotated("bl", angles_bl, &feet_rel_pos_bl);
    this->getFeetFromBaseRotated("br", angles_br, &feet_rel_pos_br);





    // std::cout << "relative fl pos " << feet_rel_pos_fl.x() << " "  << feet_rel_pos_fl.y() << " "  << feet_rel_pos_fl.z() << std::endl;
    // std::cout << "relative fr pos " << feet_rel_pos_fr.x() << " "  << feet_rel_pos_fr.y() << " "  << feet_rel_pos_fr.z() << std::endl;
    // std::cout << "relative bl pos " << feet_rel_pos_bl.x() << " "  << feet_rel_pos_bl.y() << " "  << feet_rel_pos_bl.z() << std::endl;
    // std::cout << "relative br pos " << feet_rel_pos_br.x() << " "  << feet_rel_pos_br.y() << " "  << feet_rel_pos_br.z() << std::endl;

    /*

    Eigen::Vector3f ang_vel_fl, ang_vel_fr, ang_vel_bl, ang_vel_br;
    ang_vel_fl << jointVel[0], jointVel[1], jointVel[2];
    ang_vel_fr << jointVel[3], jointVel[4], jointVel[5];
    ang_vel_bl << jointVel[6], jointVel[7], jointVel[8];
    ang_vel_br << jointVel[9], jointVel[10], jointVel[11];



    Eigen::Matrix3f J_fl, J_fr, J_bl, J_br;
    this->kinematics->getJacobian("fl", angles_fl, &J_fl);
    this->kinematics->getJacobian("fr", angles_fr, &J_fr);
    this->kinematics->getJacobian("bl", angles_bl, &J_bl);
    this->kinematics->getJacobian("br", angles_br, &J_br);

    Eigen::Matrix<double, 3, 1> cross_fl, cross_fr, cross_bl, cross_br;
    crossProduct(this->angular_vel,  feet_rel_pos_fl.cast<double>(),   &cross_fl);
    crossProduct(this->angular_vel,  feet_rel_pos_fr.cast<double>(),   &cross_fr);
    crossProduct(this->angular_vel,  feet_rel_pos_bl.cast<double>(),   &cross_bl);
    crossProduct(this->angular_vel,  feet_rel_pos_br.cast<double>(),   &cross_br);

    Eigen::Matrix<double, 3, 1> feet_rel_vel_fl, feet_rel_vel_fr, feet_rel_vel_bl, feet_rel_vel_br;
    feet_rel_vel_fl  = cross_fl + this->base_rotation_matrix * J_fl.cast<double>() * ang_vel_fl.cast<double>();
    feet_rel_vel_fr  = cross_fr + this->base_rotation_matrix * J_fr.cast<double>() * ang_vel_fr.cast<double>();
    feet_rel_vel_bl  = cross_bl + this->base_rotation_matrix * J_bl.cast<double>() * ang_vel_bl.cast<double>();
    feet_rel_vel_br  = cross_br + this->base_rotation_matrix * J_br.cast<double>() * ang_vel_br.cast<double>();

    */

    Eigen::Matrix<double, 3, 1> cross_fl, cross_fr, cross_bl, cross_br;
    crossProduct(this->angular_vel,  feet_rel_pos_fl.cast<double>(),   &cross_fl);
    crossProduct(this->angular_vel,  feet_rel_pos_fr.cast<double>(),   &cross_fr);
    crossProduct(this->angular_vel,  feet_rel_pos_bl.cast<double>(),   &cross_bl);
    crossProduct(this->angular_vel,  feet_rel_pos_br.cast<double>(),   &cross_br);

    Eigen::Matrix<double, 3, 1> feet_rel_vel_fl, feet_rel_vel_fr, feet_rel_vel_bl, feet_rel_vel_br;
    feet_rel_vel_fl  = cross_fl - this->foot_vel_fl.cast<double>();
    feet_rel_vel_fr  = cross_fr - this->foot_vel_fr.cast<double>();
    feet_rel_vel_bl  = cross_bl - this->foot_vel_bl.cast<double>();
    feet_rel_vel_br  = cross_br - this->foot_vel_br.cast<double>();

    // feet_rel_vel_fl  = -this->foot_vel_fl.cast<double>();
    // feet_rel_vel_fr  = -this->foot_vel_fr.cast<double>();
    // feet_rel_vel_bl  = -this->foot_vel_bl.cast<double>();
    // feet_rel_vel_br  = -this->foot_vel_br.cast<double>();


    // feet_rel_vel_fl << -0.2,0,0;
    // feet_rel_vel_fr << -0.2,0,0;
    // feet_rel_vel_bl << -0.2,0,0;
    // feet_rel_vel_br << -0.2,0,0;
    // std::cout << " feet rel vel " << feet_rel_vel_fl.x() << " " << feet_rel_vel_fl.y() << " "<< feet_rel_vel_fl.z() << std::endl;

    // std::cout << "relative fl vel " << feet_rel_vel_fl.x() << " "  << feet_rel_vel_fl.y() << " "  << feet_rel_vel_fl.z() << std::endl;
    // std::cout << "relative fr vel " << feet_rel_vel_fr.x() << " "  << feet_rel_vel_fr.y() << " "  << feet_rel_vel_fr.z() << std::endl;
    // std::cout << "relative bl vel " << feet_rel_vel_bl.x() << " "  << feet_rel_vel_bl.y() << " "  << feet_rel_vel_bl.z() << std::endl;
    // std::cout << "relative br vel " << feet_rel_vel_br.x() << " "  << feet_rel_vel_br.y() << " "  << feet_rel_vel_br.z() << std::endl;


    Eigen::Matrix<double, 4, 1> feetHeight;
    feetHeight << feetHeight_[0], feetHeight_[1], feetHeight_[2], feetHeight_[3];




    this->z << -feet_rel_pos_fl.cast<double>(),
               -feet_rel_pos_fr.cast<double>(),
               -feet_rel_pos_bl.cast<double>(),
               -feet_rel_pos_br.cast<double>(),
               -feet_rel_vel_fl,
               -feet_rel_vel_fr,
               -feet_rel_vel_bl,
               -feet_rel_vel_br,
               -feetHeight;

    Eigen::Matrix3d Qpos_fl, Qpos_fr, Qpos_bl, Qpos_br;
    this->getRelativeFeetPositionCovariance(feet_stance[0], &Qpos_fl);
    this->getRelativeFeetPositionCovariance(feet_stance[1], &Qpos_fr);
    this->getRelativeFeetPositionCovariance(feet_stance[2], &Qpos_bl);
    this->getRelativeFeetPositionCovariance(feet_stance[3], &Qpos_br);


    Eigen::Matrix3d Qvel_fl, Qvel_fr, Qvel_bl, Qvel_br;
    this->getRelativeFeetVelocityCovariance(feet_stance[0], &Qvel_fl);
    this->getRelativeFeetVelocityCovariance(feet_stance[1], &Qvel_fr);
    this->getRelativeFeetVelocityCovariance(feet_stance[2], &Qvel_bl);
    this->getRelativeFeetVelocityCovariance(feet_stance[3], &Qvel_br);

    Eigen::Matrix4d Qheight;
    Qheight << getFeetHeightCovariance(this->feet_stance[0]), 0, 0, 0,
               0, getFeetHeightCovariance(this->feet_stance[1]), 0, 0,
               0, 0, getFeetHeightCovariance(this->feet_stance[2]), 0,
               0, 0, 0, getFeetHeightCovariance(this->feet_stance[3]);

    Eigen::Matrix<double, 3, 4> c;
    Eigen::Matrix<double, 4, 3> cT;
    c.fill(0);
    cT.fill(0);

    Eigen::Matrix<double, 3, MEAS_DIM> R0, R1, R2, R3, R4, R5, R6, R7;
    Eigen::Matrix<double, 4, MEAS_DIM> R8;

    R0 <<       Qpos_fl , O    , O    , O    , O    , O    , O    , O    , c;
    R1 <<       O    , Qpos_fr , O    , O    , O    , O    , O    , O    , c;
    R2 <<       O    , O    , Qpos_bl , O    , O    , O    , O    , O    , c;
    R3 <<       O    , O    , O    , Qpos_br , O    , O    , O    , O    , c;
    R4 <<       O    , O    , O    , O    , Qvel_fl , O    , O    , O    , c;
    R5 <<       O    , O    , O    , O    , O    , Qvel_fr , O    , O    , c;
    R6 <<       O    , O    , O    , O    , O    , O    , Qvel_bl , O    , c;
    R7 <<       O    , O    , O    , O    , O    , O    , O    , Qvel_br , c;
    R8 <<       cT   , cT   , cT   , cT   , cT   , cT   , cT   , cT   , Qheight;


    /*
    R0 <<       Qpos , O    , O    , O    ,  c;
    R1 <<       O    , Qpos , O    , O    ,  c;
    R2 <<       O    , O    , Qpos , O    ,  c;
    R3 <<       O    , O    , O    , Qpos ,  c;
    R8 <<       cT   , cT   , cT   , cT   , Qheight;
    */



    this->R << R0, R1, R2, R3, R4, R5, R6, R7, R8;
    // this->R << R0, R1, R2, R3, R8;

}

void StateEstimation::updateState() {
    Eigen::Matrix<double, MEAS_DIM, 1> y;
    y = this->z - this->H * this->x;

     /*
    std::cout << "P:" << std::endl;
    for (int i=0; i < STATE_DIM; i++){
        for (int j=0; j < STATE_DIM; j++){
            std::cout << this->P(i,j) << ", ";
        }
        std::cout << std::endl;
    }
    */

    /*
    std::cout << "R:" << std::endl;
    for (int i=0; i < MEAS_DIM; i++){
        for (int j=0; j < MEAS_DIM; j++){
            std::cout << this->R(i,j) << ", ";
        }
        std::cout << std::endl;
    }
    */



    Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> inv_;
    inv_ = this->H * this->P * (this->H.transpose()) + this->R;

    /*
    std::cout << "before inversion:" << std::endl;
    for (int i=0; i < MEAS_DIM; i++){
        for (int j=0; j < MEAS_DIM; j++){
            std::cout << inv_(i,j) << ", ";
        }
        std::cout << std::endl;
    }
    */


    // HERE IT BECOMES NAN BECAUSE THE inv_ MATRIX IS SOMEHOW WRONG!
    inv_ = inv_.inverse();

    /*
    std::cout << "inverse:" << std::endl;
    for (int i=0; i < MEAS_DIM; i++){
        for (int j=0; j < MEAS_DIM; j++){
            std::cout << inv_(i,j) << ", ";
    // if (isnan(inv_(i,j))){
    // return;
    // }
        }
        std::cout << std::endl;
    }
    */


    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> Kalman_gain;



    Kalman_gain = this->P * (this->H.transpose()) * inv_;
    this->x = this->x + Kalman_gain * y;
    this->P = (this->I_state_dim - Kalman_gain * this->H)*this->P;
}



void StateEstimation::getBasePosition(Eigen::Vector3f* base_pos){
    Eigen::Vector3f base_pos_;
    base_pos_ << this->x(0,0), this->x(1,0), this->x(2,0);
    base_pos->operator=(base_pos_);
}

void StateEstimation::getBaseVelocity(Eigen::Vector3f* base_vel){
    Eigen::Vector3f base_vel_;
    base_vel_ << this->x(3,0), this->x(4,0), this->x(5,0);
    base_vel->operator=(base_vel_);
}


void StateEstimation::getBaseAngularVelocity(Eigen::Vector3f* base_angular_vel){
    base_angular_vel->operator=(  this->angular_vel.cast<float>() );
}


void StateEstimation::getFeetPosition(std::string leg, Eigen::Vector3f* feet_pos){
    Eigen::Vector3f feet_pos_;
    float z_pos;
    if (leg == "fl"){
        z_pos =  this->x(8,0);
        // if (z_pos < 0) z_pos = 0;
        feet_pos_ << this->x(6,0), this->x(7,0), z_pos;
    }
    else if (leg == "fr"){
        z_pos =  this->x(11,0);
        // if (z_pos < 0) z_pos = 0;
        feet_pos_ << this->x(9,0), this->x(10,0), z_pos;
    }
    else if (leg == "bl"){
        z_pos =  this->x(14,0);
        // if (z_pos < 0) z_pos = 0;
        feet_pos_ << this->x(12,0), this->x(13,0), z_pos;
    }
    else if (leg == "br"){
        z_pos =  this->x(17,0);
        // if (z_pos < 0) z_pos = 0;
        feet_pos_ << this->x(15,0), this->x(16,0), z_pos;
    }

    feet_pos->operator=( feet_pos_ );
}


void StateEstimation::getBaseVelocityFromBaseRef(Eigen::Vector3f* base_vel){
    Eigen::Vector3f base_vel_;
    base_vel_ << this->x(3,0), this->x(4,0), this->x(5,0);
    base_vel->operator=(this->base_rotation_matrix.transpose().cast<float>() * base_vel_);
}



void StateEstimation::getBaseAngularVelocityFromBaseRef(Eigen::Vector3f* base_ang_vel){
    Eigen::Vector3f base_vel_;
    base_ang_vel->operator=(this->base_rotation_matrix.transpose().cast<float>() * this->angular_vel.cast<float>());
}

void StateEstimation::getBaseRotationMatrix(Eigen::Matrix3f* base_rot_matrix){
    base_rot_matrix->operator=( this->base_rotation_matrix.cast<float>()  );
}


void StateEstimation::getBaseAccelerationFromBaseRef(Eigen::Vector3f* base_acc){
    base_acc->operator=(this->acc.cast<float>());
}

float StateEstimation::getMeanFeetHeight(){
    float feet_fl_z, feet_fr_z, feet_bl_z, feet_br_z;
    feet_fl_z = this->x(8,0);
    feet_fr_z = this->x(11,0);
    feet_bl_z = this->x(14,0);
    feet_br_z = this->x(17,0);

    float meanz = this->feet_stance[0]*feet_fl_z +
                  this->feet_stance[1]*feet_fr_z +
                  this->feet_stance[2]*feet_bl_z +
                  this->feet_stance[3]*feet_br_z;

    return meanz;

}


void StateEstimation::getCoMPosition(Eigen::Vector3f* com_pos){
    /*
    Eigen::Vector3f com_pos_tmp;

    Eigen::Vector3f feet_fl, feet_fr, feet_bl, feet_br, base_pos;


    this->getFeetPosition("fl", &feet_fl);
    this->getFeetPosition("fr", &feet_fr);
    this->getFeetPosition("bl", &feet_bl);
    this->getFeetPosition("br", &feet_br);
    this->getBasePosition(&base_pos);


    feet_fl = this->base_rotation_matrix.transpose() * (feet_fl - base_pos);
    feet_fr = this->base_rotation_matrix.transpose() * (feet_fr - base_pos);
    feet_bl = this->base_rotation_matrix.transpose() * (feet_bl - base_pos);
    feet_br = this->base_rotation_matrix.transpose() * (feet_br - base_pos);


    com_pos_tmp = this->feet_stance[0]*feet_fl +
                  this->feet_stance[1]*feet_fr +
                  this->feet_stance[2]*feet_bl +
                  this->feet_stance[3]*feet_br;

    com_pos_tmp << com_pos_tmp.x(), com_pos_tmp.y(), base_pos.z();

    // std::cout << com_pos_tmp.x() << " " << com_pos_tmp.y() << " " << com_pos_tmp.z() << std::endl;

    com_pos->operator=(com_pos_tmp);
    */
}
