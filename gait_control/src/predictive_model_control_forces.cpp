//
// Created by andres on 20.03.21.
//
#include "../include/predictive_model_control.h"

PredictiveModeController::PredictiveModeController(FeetTrajectory* fl_trajectory, FeetTrajectory* fr_trajectory, FeetTrajectory* bl_trajectory, FeetTrajectory* br_trajectory, Kinematics* kinematics){
    this->fl_trajectory = fl_trajectory;
    this->fr_trajectory = fr_trajectory;
    this->bl_trajectory = bl_trajectory;
    this->br_trajectory = br_trajectory;

    this->kinematics = kinematics;


    int horizon = 1;
    double dt    = 0.1;
    this->setHorizon_and_dtime(horizon, dt);

    this->I3 << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;

    this->O3 << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;

    this->O3H << 0, 0, 0;
    this->O3V << 0, 0, 0;

    this->base_CoM_offset << 0,0,0;

    // Eigen::Matrix3d Inertia;

    this->Inertia << ROBOT_INERTIA_XX, ROBOT_INERTIA_XY, ROBOT_INERTIA_XZ,
                     ROBOT_INERTIA_XY, ROBOT_INERTIA_YY, ROBOT_INERTIA_YZ,
                     ROBOT_INERTIA_XZ, ROBOT_INERTIA_YZ, ROBOT_INERTIA_ZZ;


    this->state_ref.setZero();
    this->initial_state.setZero();

    this->base_pos.setZero();
    this->base_ang.setZero();
    this->base_vel.setZero();
    this->base_ang_vel.setZero();

    this->base_vel_weight.setZero();

    this->w_dyn_model.setZero();
    this->w_forces    = 0;



    this->stance_fl = true;
    this->stance_fr = true;
    this->stance_bl = true;
    this->stance_br = true;

    this->terrain_fl =  Eigen::MatrixXd::Zero(5, 3);
    this->terrain_fr =  Eigen::MatrixXd::Zero(5, 3);
    this->terrain_bl =  Eigen::MatrixXd::Zero(5, 3);
    this->terrain_br =  Eigen::MatrixXd::Zero(5, 3);

    this->d_min << 0,0,0,0,0;
    this->d_max << 0,0,0,0,0;

    this->curr_forces.resize(12);
    this->curr_forces.setZero();




    /*
   this->solve_qp = new SQProblem(HORIZON*4*3, HORIZON*4*5) ;
   this->init_qp = false;

    Options QP_options;
    QP_options.setToMPC( );
    QP_options.printLevel = PL_LOW;
    this->solve_qp->setOptions( QP_options );
    */
}

PredictiveModeController::~PredictiveModeController(){

}

void PredictiveModeController::setHorizon_and_dtime(int horizon, double dt){
    this->HORIZON = horizon;
    this->DELTAT  = dt;

    this->state_ref.resize(  DYN_STATE_DIM * this->HORIZON,  1  );

}


void PredictiveModeController::setBaseCoMOffset(double base_offset_x, double base_offset_y){
    base_CoM_offset << base_offset_x, base_offset_y, 0;
}


void PredictiveModeController::getCrossProductMatrix(Eigen::Vector3d r, Eigen::Matrix3d* C){
    Eigen::Matrix3d C_;
    C_ <<  0,     -r.z(),  r.y(),
           r.z(),  0,     -r.x(),
          -r.y(),  r.x(),  0;
    C->operator=( C_ );
}

void PredictiveModeController::getRz(double yaw, Eigen::Matrix3d* Rz){
    Eigen::Matrix3d Rz_;
    Rz_ <<   cos(yaw), -sin(yaw),   0,
             sin(yaw),  cos(yaw),   0,
             0,            0,       1;

    Rz->operator=( Rz_ );

}


void PredictiveModeController::setFeetPos(std::string leg, const Eigen::Vector3d &feet_pos){
    if (leg == "fl") this->feet_pos_fl = feet_pos;
    if (leg == "fr") this->feet_pos_fr = feet_pos;
    if (leg == "bl") this->feet_pos_bl = feet_pos;
    if (leg == "br") this->feet_pos_br = feet_pos;
}


void PredictiveModeController::setTerrainPlane(std::string leg, double mu, const Eigen::Vector3d &normal, const Eigen::Vector3d &tangent1, const Eigen::Vector3d &tangent2, double yaw){


    Eigen::Matrix3d Rz;

    this->getRz(yaw, &Rz);
    Eigen::Matrix<double, 3, 1> t1, t2;

    t1 = Rz * tangent1;
    t2 = Rz * tangent2;

    // std::cout << yaw << std::endl;
    // std::cout << "t1 = " << t1.x() << " " << t1.y() << " " << t1.z() << std::endl;
    // std::cout << "t1 = " << t2.x() << " " << t2.y() << " " << t2.z() << std::endl;
    // std::cout << "n  = " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;

    // std::cout << "-------" << std::endl;



    Eigen::Matrix<double,5,3> terrain_mat;

    /*
    terrain_mat <<  -mu * normal.x() + tangent1.x(),  -mu * normal.y() + tangent1.y(), -mu * normal.z() + tangent1.z(),
                    -mu * normal.x() + tangent2.x(),  -mu * normal.y() + tangent2.y(), -mu * normal.z() + tangent2.z(),
                     mu * normal.x() + tangent1.x(),   mu * normal.y() + tangent1.y(),  mu * normal.z() + tangent1.z(),
                     mu * normal.x() + tangent2.x(),   mu * normal.y() + tangent2.y(),  mu * normal.z() + tangent2.z(),
                     normal.x(),                       normal.y(),                      normal.z();

    */

    terrain_mat <<  -mu * normal.x() + t1.x(),  -mu * normal.y() + t1.y(), -mu * normal.z() + t1.z(),
                    -mu * normal.x() + t2.x(),  -mu * normal.y() + t2.y(), -mu * normal.z() + t2.z(),
                    mu * normal.x() + t1.x(),    mu * normal.y() + t1.y(),  mu * normal.z() + t1.z(),
                    mu * normal.x() + t2.x(),    mu * normal.y() + t2.y(),  mu * normal.z() + t2.z(),
                    normal.x(),                        normal.y(),                      normal.z();




    if (leg == "fl") this->terrain_fl = terrain_mat;
    if (leg == "fr") this->terrain_fr = terrain_mat;
    if (leg == "bl") this->terrain_bl = terrain_mat;
    if (leg == "br") this->terrain_br = terrain_mat;


}


void PredictiveModeController::setFeetForceLimits(double min_force, double max_force){


    this->d_min << - 1000000000,  // large negative number, in theory - inf
                   - 1000000000,  // large negative number, in theory - inf
                   0,
                   0,
                   min_force;

    this->d_max <<   0,
                     0,
                     1000000000,  // large positive number, in theory  inf
                     1000000000,  // large positive number, in theory  inf
                     max_force;
}


void PredictiveModeController::setWeightsController(const Eigen::Vector3d &base_pos_weight, const Eigen::Vector3d &base_ang_weight, const Eigen::Vector3d &base_vel_weight, const Eigen::Vector3d &base_ang_vel_weight,  double w_forces){


    this->w_dyn_model << base_ang_weight.x(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, base_ang_weight.y(), 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0, 0, 0, 0, 0,
                         0, 0, base_ang_weight.z(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, base_pos_weight.x(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, base_pos_weight.y(), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, base_pos_weight.z(), 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, base_ang_vel_weight.x(), 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, base_ang_vel_weight.y(), 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, base_ang_vel_weight.z(), 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, base_vel_weight.x(), 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, base_vel_weight.y(), 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, base_vel_weight.z(), 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

            this->w_forces        = w_forces;
            this->base_vel_weight = base_vel_weight;
}





void PredictiveModeController::getAmatrix(double yaw,
                                          const Eigen::Matrix3d &InertiaInv,
                                          const Eigen::Vector3d &pos_fl,
                                          const Eigen::Vector3d &pos_fr,
                                          const Eigen::Vector3d &pos_bl,
                                          const Eigen::Vector3d &pos_br,
                                          bool*  stance_feets,
                                          Matrix13d* A){

    Eigen::Matrix3d Rz, RzT;
    this->getRz(yaw, &Rz);
    RzT = Rz.transpose();
    Eigen::Matrix3d IG;





    Eigen::Matrix3d fl_x, fr_x, bl_x, br_x, K;
    this->getCrossProductMatrix(pos_fl, &fl_x);
    this->getCrossProductMatrix(pos_fr, &fr_x);
    this->getCrossProductMatrix(pos_bl, &bl_x);
    this->getCrossProductMatrix(pos_br, &br_x);

    int num_stance_feet = stance_feets[0] + stance_feets[1] + stance_feets[2] + stance_feets[3];

    if (num_stance_feet == 0){
        K.setZero();
        IG.setIdentity();  // Gravity contribution
    }

    if (num_stance_feet < 4){
        float m_div_feet = ROBOT_TOTAL_MASS / num_stance_feet;
        K = - InertiaInv * (stance_feets[0]*fl_x + stance_feets[1]*fr_x + stance_feets[2]*bl_x + stance_feets[3]*br_x) * m_div_feet;
        IG.setZero();

        // K = stance_feets[0]*fl_x + stance_feets[1]*fr_x + stance_feets[2]*bl_x + stance_feets[3]*br_x;

    }

    if (num_stance_feet > 3){
        K.setZero();
        IG.setZero();

    }
    // K.setZero();

    /*
    std::cout << "-------Matrix K:------"<< std::endl;
    std::cout << K(0,0) << " " << K(0,1) << " " << K(0,2) << std::endl;
    std::cout << K(1,0) << " " << K(1,1) << " " << K(1,2) << std::endl;
    std::cout << K(2,0) << " " << K(2,1) << " " << K(2,2) << std::endl;
    std::cout << "---------------------" << std::endl;


    K = -(stance_feets[0]*fl_x + stance_feets[1]*fr_x + stance_feets[2]*bl_x + stance_feets[3]*br_x) * m_div_feet;
    std::cout << "-------Matrix K:------"<< std::endl;
    std::cout << K(0,0) << " " << K(0,1) << " " << K(0,2) << std::endl;
    std::cout << K(1,0) << " " << K(1,1) << " " << K(1,2) << std::endl;
    std::cout << K(2,0) << " " << K(2,1) << " " << K(2,2) << std::endl;
    std::cout << "---------------------" << std::endl;


    K.setZero();
    */

    IG.setIdentity();  // Gravity contribution

    Eigen::Matrix<double, 3, DYN_STATE_DIM> A0, A1, A2, A3, A4;

    A0 << O3,  O3,  RzT,  O3,  O3;
    A1 << O3,  O3,  O3,   I3,  O3;
    A2 << O3,  O3,  O3,   O3,  O3;
    A3 << O3,  O3,  O3,   O3,  I3;
    A4 << O3,  O3,  O3,   O3,  I3;

    // A0 << O3,  O3,  RzT,  O3,  O3;
    // A1 << O3,  O3,  O3,   I3,  O3;
    // A2 << O3,  O3,  O3,   O3,  O3;
    // A3 << O3,  O3,  O3,   O3,  I3;
    // A4 << O3,  O3,  O3,   O3,  O3;

    Matrix13d A_;
    A_ << A0, A1, A2, A3, A4;
    A->operator=(A_);
}


void PredictiveModeController::getBmatrix(double yaw,
                                          const Eigen::Matrix3d &InertiaInv,
                                          const Eigen::Vector3d &pos_fl,
                                          const Eigen::Vector3d &pos_fr,
                                          const Eigen::Vector3d &pos_bl,
                                          const Eigen::Vector3d &pos_br,
                                          Matrix13x12d* B){
    // Eigen::Matrix3f Rz, InertiaTrans, InertiaInv;
    // this->getRz(yaw, &Rz);
    // InertiaTrans = Rz * this->Inertia * Rz.transpose();
    // InertiaInv = InertiaTrans.inverse();

    Eigen::Matrix3d fl_x, fr_x, bl_x, br_x, Idivm;
    this->getCrossProductMatrix(pos_fl, &fl_x);
    this->getCrossProductMatrix(pos_fr, &fr_x);
    this->getCrossProductMatrix(pos_bl, &bl_x);
    this->getCrossProductMatrix(pos_br, &br_x);

    Idivm =  I3 * (1.0/ROBOT_TOTAL_MASS);


    Eigen::Matrix<double, 3, 12> B0, B1, B2, B3, B4;
    Eigen::Matrix<double, 1, 12> B5;


    B0 <<  O3,                  O3,                  O3,                  O3;
    B1 <<  O3,                  O3,                  O3,                  O3;
    B2 << InertiaInv * fl_x,  InertiaInv * fr_x, InertiaInv * bl_x,  InertiaInv * br_x;
    B3 << Idivm,              Idivm,              Idivm,             Idivm;
    B4 <<  O3,                  O3,                  O3,                  O3;


    // B0 <<  O3,                  O3,                  O3,                  O3;
    // B1 <<  O3,                  O3,                  O3,                  O3;
    // B2 << InertiaInv * fl_x,  InertiaInv * fr_x,  InertiaInv * bl_x,  InertiaInv * br_x;
    // B3 << Idivm,              Idivm,              Idivm,              Idivm;
    // B4 <<  O3,                  O3,                  O3,                  O3;

    Matrix13x12d B_;
    B_ << B0, B1, B2, B3, B4;

    B->operator=(  B_  );
}

void PredictiveModeController::getAdynAndBdynMatrices(double dt, const Matrix13d &A, const Matrix13x12d &B, Matrix13d *Adyn, Matrix13x12d* Bdyn){
    Eigen::MatrixXd  A_ext(DYN_STATE_DIM * 12, DYN_STATE_DIM * 12);
    Eigen::MatrixXd eA_ext(DYN_STATE_DIM * 12, DYN_STATE_DIM * 12);

    A_ext.setZero();
    A_ext.block(0,0, DYN_STATE_DIM, DYN_STATE_DIM) = A*dt;
    A_ext.block(0,DYN_STATE_DIM, DYN_STATE_DIM, 12) = B*dt;
    eA_ext = A_ext.exp();

    Adyn->operator=(  eA_ext.block(0,0,DYN_STATE_DIM, DYN_STATE_DIM)  );
    Bdyn->operator=(  eA_ext.block(0,DYN_STATE_DIM,DYN_STATE_DIM, 12)  );
}



void PredictiveModeController::getAdynMatrix(double yaw,  double dt,
                                             const Eigen::Matrix3d &InertiaInv,
                                             const Eigen::Vector3d &pos_fl,
                                             const Eigen::Vector3d &pos_fr,
                                             const Eigen::Vector3d &pos_bl,
                                             const Eigen::Vector3d &pos_br,
                                             bool*  stance_feets,
                                             Matrix13d* Adyn){
    Matrix13d A, I13;
    I13.setIdentity();
    this->getAmatrix(yaw,
                     InertiaInv,
                     pos_fl,
                     pos_fr,
                     pos_bl,
                     pos_br,
                     stance_feets,
                     &A);

    Adyn->operator=(  I13 + A*dt );
}


void PredictiveModeController::getBdynMatrix(double yaw, double dt,
                                             const Eigen::Matrix3d &InertiaInv,
                                             const Eigen::Vector3d &pos_fl,
                                             const Eigen::Vector3d &pos_fr,
                                             const Eigen::Vector3d &pos_bl,
                                             const Eigen::Vector3d &pos_br,
                                             Matrix13x12d* Bdyn){
    Matrix13x12d B;
    this->getBmatrix(yaw, InertiaInv, pos_fl, pos_fr, pos_bl, pos_br, &B);

    Bdyn->operator=(  B*dt  );
}





void PredictiveModeController::setRobotState(const Eigen::Vector3d base_pos, const Eigen::Vector3d base_vel, const Eigen::Vector3d base_ang, const Eigen::Vector3d base_ang_vel){
    Eigen::Vector3d gravity;
    gravity << 0,0, -9.8;

    this->initial_state << base_ang, base_pos, base_ang_vel, base_vel, gravity;

    this->base_pos     = base_pos;
    this->base_ang     = base_ang;
    this->base_vel     = base_vel;
    this->base_ang_vel = base_ang_vel;
    /*
    for(int i=0; i<13; i++){
        std::cout <<   this->initial_state(i) << ", ";
    }
    std::cout << std::endl;
    */

}


void PredictiveModeController::setRobotControl(const Eigen::Vector3d des_base_vel, const Eigen::Vector3d des_base_ang_vel) {
    Eigen::Vector3d base_ang_, des_base_ang_vel_;

    base_ang_ << 0, 0, this->base_ang.z();          // Pitch and roll set to zero
    des_base_ang_vel_ << 0, 0, des_base_ang_vel.z();


    Eigen::Vector3d integrated_base_pos, integrated_base_ang;
    integrated_base_ang = base_ang_ ;
    integrated_base_pos = this->base_pos;

    Eigen::Vector3d gravity;
    gravity << 0,0, -9.8;

    for (int k = 0; k < HORIZON; k++) {
        integrated_base_ang = integrated_base_ang + DELTAT * des_base_ang_vel_;
        integrated_base_pos = integrated_base_pos + DELTAT * des_base_vel;


        this->state_ref.block<3, 1>(DYN_STATE_DIM * k, 0)      = base_ang_ + (k) * DELTAT * des_base_ang_vel_;
        this->state_ref.block<3, 1>(DYN_STATE_DIM * k + 3, 0)  = this->base_pos + (k) * DELTAT * des_base_vel;
        this->state_ref.block<3, 1>(DYN_STATE_DIM * k + 6, 0)  = des_base_ang_vel_;
        this->state_ref.block<3, 1>(DYN_STATE_DIM * k + 9, 0)  = des_base_vel;
        this->state_ref.block<3, 1>(DYN_STATE_DIM * k + 12, 0) = gravity;


        // std::cout << this->state_ref(DYN_STATE_DIM*k+3,0) << ", ";
    }
    // std::cout << std::endl;
    /*
    std::cout << "------------------" << std::endl;
    for (int k = 0; k < HORIZON * DYN_STATE_DIM; k++) {
        std::cout << this->state_ref(k, 0) << ", ";
    }
    std::cout << std::endl;
    */
}



void PredictiveModeController::computeForces(double current_main_phase){
    Matrix13d    A, Adyn, Adyn_prev;
    Matrix13x12d B, Bdyn;

    Adyn_prev = Eigen::MatrixXd::Identity(DYN_STATE_DIM, DYN_STATE_DIM);

    Eigen::Matrix4d trans_body_matrix;

    Eigen::Vector3d ref_base_ang, ref_base_pos, ref_base_ang_vel, ref_base_vel;
    Eigen::Vector3d feet_pos_fl, feet_pos_fr, feet_pos_bl, feet_pos_br;
    Eigen::Vector3d feet_pos_fl_fromBase, feet_pos_fr_fromBase, feet_pos_bl_fromBase, feet_pos_br_fromBase;
    Eigen::Vector3d feet_pos_fl_fromWorld, feet_pos_fr_fromWorld, feet_pos_bl_fromWorld, feet_pos_br_fromWorld;
    Eigen::Vector3d pos_fl, pos_fr, pos_bl, pos_br;

    Eigen::Vector3d normal_force;

    bool stance_fl_, stance_fr_, stance_bl_, stance_br_;
    int num_stance_feet, num_stance_feet_k0, offset_feet;

    // Big matrices for QP optimisation
    Eigen::MatrixXd Aqp(HORIZON * DYN_STATE_DIM, DYN_STATE_DIM);
    Eigen::MatrixXd A_consecutive(HORIZON * DYN_STATE_DIM, DYN_STATE_DIM);

    Eigen::MatrixXd Bqp(HORIZON * DYN_STATE_DIM, 0);
    Eigen::MatrixXd Cqp(0, 0);
    Eigen::MatrixXd D_MINqp(0, 1);
    Eigen::MatrixXd D_MAXqp(0, 1);
    Eigen::MatrixXd NORMALS(0, 1);

    Eigen::MatrixXd L(HORIZON * DYN_STATE_DIM, HORIZON * DYN_STATE_DIM); // weights dynamical model
    L.setZero();

    Aqp.setZero();
    A_consecutive.setZero();
    Bqp.setZero();
    Cqp.setZero();
    D_MINqp.setZero();
    D_MAXqp.setZero();

    Eigen::VectorXd num_stance_feet_it(HORIZON);



    int forces_counter = 0;
    bool stance_feet[4];


    // std::cout << "--------------------------------REF--------------------------------" << std::endl;

    for(int k=0; k<HORIZON; k++) {
        ref_base_ang     << this->state_ref(DYN_STATE_DIM*k),    this->state_ref(DYN_STATE_DIM*k + 1),  this->state_ref(DYN_STATE_DIM*k + 2);
        ref_base_pos     << this->state_ref(DYN_STATE_DIM*k+3),  this->state_ref(DYN_STATE_DIM*k + 4),  this->state_ref(DYN_STATE_DIM*k + 5);
        ref_base_ang_vel << this->state_ref(DYN_STATE_DIM*k+6),  this->state_ref(DYN_STATE_DIM*k + 7),  this->state_ref(DYN_STATE_DIM*k + 8);
        ref_base_vel     << this->state_ref(DYN_STATE_DIM*k+9),  this->state_ref(DYN_STATE_DIM*k + 10), this->state_ref(DYN_STATE_DIM*k + 11);

        if (ref_base_vel.norm() > 0){
        /*
                std::cout << "reference " << k << ":" << std::endl;
                std::cout << "base_pos: " << ref_base_pos.x() << " " << ref_base_pos.y() << " " << ref_base_pos.z() << std::endl;
                std::cout << "base_vel: " << ref_base_vel.x() << " " << ref_base_vel.y() << " " << ref_base_vel.z() << std::endl;
                std::cout << "base_ang: " << ref_base_ang.x() << " " << ref_base_ang.y() << " " << ref_base_ang.z() << std::endl;
                std::cout << "ang_vel:  " << ref_base_ang_vel.x() << " " << ref_base_ang_vel.y() << " " << ref_base_ang_vel.z() << std::endl << std::endl;

        */
        }



        // position printouts for pos_fl/pos_fr can drift even when the robot is static.
        stance_fl_ = this->fl_trajectory->predictFeetState(current_main_phase, (k)*DELTAT, ref_base_pos, ref_base_vel, ref_base_ang, ref_base_ang_vel, this->feet_pos_fl, &feet_pos_fl_fromWorld);
        stance_fr_ = this->fr_trajectory->predictFeetState(current_main_phase, (k)*DELTAT, ref_base_pos, ref_base_vel, ref_base_ang, ref_base_ang_vel, this->feet_pos_fr, &feet_pos_fr_fromWorld);
        stance_bl_ = this->bl_trajectory->predictFeetState(current_main_phase, (k)*DELTAT, ref_base_pos, ref_base_vel, ref_base_ang, ref_base_ang_vel, this->feet_pos_bl, &feet_pos_bl_fromWorld);
        stance_br_ = this->br_trajectory->predictFeetState(current_main_phase, (k)*DELTAT, ref_base_pos, ref_base_vel, ref_base_ang, ref_base_ang_vel, this->feet_pos_br, &feet_pos_br_fromWorld);
        //
        // std::cout << "fl_pos: " << feet_pos_fl_fromWorld.x() << " " << feet_pos_fl_fromWorld.y() << " " << feet_pos_fl_fromWorld.z() << std::endl;



        stance_feet[0] = stance_fl_;
        stance_feet[1] = stance_fr_;
        stance_feet[2] = stance_bl_;
        stance_feet[3] = stance_br_;


        // feet_pos_fl_fromWorld << 0.1,0.1,-0.15;
        // feet_pos_fr_fromWorld << 0.1,-0.1,-0.15;
        // feet_pos_bl_fromWorld << -0.1,0.1,-0.15;
        // feet_pos_br_fromWorld << -0.1,-0.1,-0.15;


        pos_fl = feet_pos_fl_fromWorld - ref_base_pos + base_CoM_offset;
        pos_fr = feet_pos_fr_fromWorld - ref_base_pos + base_CoM_offset;
        pos_bl = feet_pos_bl_fromWorld - ref_base_pos + base_CoM_offset;
        pos_br = feet_pos_br_fromWorld - ref_base_pos + base_CoM_offset;


        // EL PROBLEMA NO ESTA AQUI



        num_stance_feet = stance_fl_ + stance_fr_ + stance_bl_ + stance_br_;
        if (k == 0){
            num_stance_feet_k0 = num_stance_feet;
            this->stance_fl = stance_fl_;
            this->stance_fr = stance_fr_;
            this->stance_bl = stance_bl_;
            this->stance_br = stance_br_;
        }

        // std::cout << "stance feets " << num_stance_feet << std::endl;

        double yaw = ref_base_ang.z();

        Eigen::Matrix3d Rz, InertiaTrans, InertiaInv;


        this->getRz(yaw, &Rz);

        InertiaTrans = Rz * this->Inertia * Rz.transpose(); //Rz.transpose() * this->Inertia * Rz; //
        InertiaInv = InertiaTrans.inverse();

        /*
        this->getAdynMatrix(yaw, DELTAT,
                            InertiaInv,
                            pos_fl,
                            pos_fr,
                            pos_bl,
                            pos_br,
                            stance_feet,
                            &Adyn);


        this->getmMatrix(yaw, DELTAT,
                            InertiaInv,
                            pos_fl,
                            pos_fr,
                            pos_bl,
                            pos_br,
                            &Bdyn);
        */
        this->getAmatrix(yaw,
                         InertiaInv,
                         pos_fl,
                         pos_fr,
                         pos_bl,
                         pos_br,
                         stance_feet,
                         &A);

        this->getBmatrix(yaw,
                         InertiaInv,
                         pos_fl,
                         pos_fr,
                         pos_bl,
                         pos_br,
                         &B);


        Adyn = (A*DELTAT).exp(); // it's not here
        Adyn = Eigen::MatrixXd::Identity(DYN_STATE_DIM, DYN_STATE_DIM) + (A*DELTAT);;

        Adyn = Adyn_prev * Adyn;




        if(num_stance_feet == 0) {
            Adyn_prev = Adyn;
            // std::cout << "no feet stance" << std::endl;
            continue;

        }
        else{
            // std::cout << "stance" << std::endl;
            Adyn_prev = Eigen::MatrixXd::Identity(DYN_STATE_DIM, DYN_STATE_DIM);
        }

        /*********** GET ALL THE MATRICES FOR THE STANCE FEETS******************/

        Eigen::MatrixXd D_MIN(num_stance_feet*5, 1);
        Eigen::MatrixXd D_MAX(num_stance_feet*5, 1);
        Eigen::MatrixXd C(num_stance_feet*5, num_stance_feet*3);
        C.setZero();

        Eigen::MatrixXd B_red(DYN_STATE_DIM, 3 * num_stance_feet);
        Eigen::MatrixXd Bdyn_red(DYN_STATE_DIM, 3 * num_stance_feet);




        // Block starting at (i,j) of size (p,q) -----> matrix.block(i,j,p,q);
        offset_feet = 0;
        // std::cout << "stance feet " << num_stance_feet << " " << stance_fl <<  stance_fr << stance_bl <<  stance_br << std::endl;
        if(stance_fl_){
            // std::cout << "FL feet_offset " << offset_feet << std::endl;
            C.block(5*offset_feet, 3*offset_feet, 5, 3) = this->terrain_fl;
            D_MIN.block(5*offset_feet,0, 5, 1)          = this->d_min;
            D_MAX.block(5*offset_feet,0, 5, 1)          = this->d_max;

            B_red.block(0, 3*offset_feet, DYN_STATE_DIM, 3) = B.block(0,0, DYN_STATE_DIM, 3);
            offset_feet += 1;
        }


        if(stance_fr_){
            // std::cout << "FR feet_offset " << offset_feet << std::endl;
            C.block(5*offset_feet, 3*offset_feet, 5, 3) = this->terrain_fr;
            D_MIN.block(5*offset_feet,0, 5, 1)          = this->d_min;
            D_MAX.block(5*offset_feet,0, 5, 1)          = this->d_max;

            B_red.block(0, 3*offset_feet, DYN_STATE_DIM, 3) = B.block(0,3, DYN_STATE_DIM, 3);
            offset_feet += 1;
        }

        if(stance_bl_){
            // std::cout << "BL feet_offset " << offset_feet << std::endl;
            C.block(5*offset_feet, 3*offset_feet, 5, 3) = this->terrain_bl;
            D_MIN.block(5*offset_feet,0, 5, 1)          = this->d_min;
            D_MAX.block(5*offset_feet,0, 5, 1)          = this->d_max;

            B_red.block(0, 3*offset_feet, DYN_STATE_DIM, 3) = B.block(0,6, DYN_STATE_DIM, 3);
            offset_feet += 1;
        }

        if(stance_br_){
            C.block(5*offset_feet, 3*offset_feet, 5, 3) = this->terrain_br;
            D_MIN.block(5*offset_feet,0, 5, 1)          = this->d_min;
            D_MAX.block(5*offset_feet,0, 5, 1)          = this->d_max;

            B_red.block(0, 3*offset_feet, DYN_STATE_DIM, 3) = B.block(0,9, DYN_STATE_DIM, 3);
            offset_feet += 1;
        }


        // Bdyn_red = (DELTAT * Eigen::MatrixXd::Identity(DYN_STATE_DIM, DYN_STATE_DIM) + A* (pow(DELTAT, 2)/2)  )*(B_red);//A.inverse() * (Adyn - Eigen::MatrixXf::Identity(DYN_STATE_DIM, DYN_STATE_DIM)) * B;
        Bdyn_red = B_red * DELTAT;


        /**************** GET MATRICES FOR QP *********************/

        if (k ==0){
            Aqp.block(0*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM)     = Adyn;
        }
        else{
            Aqp.block(k*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) = Adyn * Aqp.block((k-1)*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM);
        }

        A_consecutive.block(k*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) = Adyn;



        Bqp.conservativeResize(Eigen::NoChange, Bqp.cols() + 3 * num_stance_feet);
        Bqp.block(0, 3 * forces_counter, DYN_STATE_DIM* HORIZON, 3 * num_stance_feet).setZero();

        for (int j=0; j < HORIZON; j ++){
        if (j == k){
                Bqp.block(j*DYN_STATE_DIM, 3 * forces_counter, DYN_STATE_DIM, 3 * num_stance_feet) = Bdyn_red;
            }
        if (j > k){
               Bqp.block(j*DYN_STATE_DIM, 3 * forces_counter, DYN_STATE_DIM, 3 * num_stance_feet) = Adyn.pow(j-k) * Bdyn_red;

            }
        }

        /*
        for (int j=0; j < HORIZON; j ++){

            if(j == k){
                Bqp.block(j*DYN_STATE_DIM, 3 * forces_counter, DYN_STATE_DIM, 3 * num_stance_feet) = Bdyn_red;
            }
            if (j > k){
                // Bqp.block(j*DYN_STATE_DIM, 3 * forces_counter, DYN_STATE_DIM, 3 * num_stance_feet) = Adyn.pow(j-k) * Bdyn_red;
                Bqp.block(j*DYN_STATE_DIM, 3 * forces_counter, DYN_STATE_DIM, 3 * num_stance_feet) = Aqp.block((j-k-1)*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) * Bdyn_red;
            }
        }
        */







        // for(int i=0; i < num_stance_feet*5; i++){
        // for(int j=0; j < num_stance_feet*3; j++){
        // std::cout << C(i,j) << " ";
        // }
        // std::cout << std::endl;
        // }
        // std::cout << std::endl;



        Cqp.conservativeResize(Cqp.rows() + 5*num_stance_feet, Cqp.cols() + 3*num_stance_feet);
        Cqp.block(5*forces_counter, 0, 5*num_stance_feet, Cqp.cols()).setZero();
        Cqp.block(0, 3*forces_counter, Cqp.rows(),  3*num_stance_feet).setZero();

        Cqp.block(5*forces_counter, 3*forces_counter, 5*num_stance_feet, 3*num_stance_feet) = C;

        D_MINqp.conservativeResize(D_MINqp.rows() + 5*num_stance_feet, Eigen::NoChange);
        D_MAXqp.conservativeResize(D_MAXqp.rows() + 5*num_stance_feet, Eigen::NoChange);

        D_MINqp.block(5*forces_counter, 0, 5*num_stance_feet, 1) = D_MIN;
        D_MAXqp.block(5*forces_counter, 0, 5*num_stance_feet, 1) = D_MAX;

        NORMALS.conservativeResize(NORMALS.rows() + 3*num_stance_feet, Eigen::NoChange);
        NORMALS.block(3*forces_counter, 0, 3*num_stance_feet, 1).setZero();
        normal_force << 0, 0, (ROBOT_TOTAL_MASS/num_stance_feet)*9.8;
        for(int i=0; i<num_stance_feet; i++){
            NORMALS.block(3*forces_counter + i*3, 0, 3, 1) = normal_force;
        }


        L.block(k*DYN_STATE_DIM, k*DYN_STATE_DIM, DYN_STATE_DIM, DYN_STATE_DIM) = this->w_dyn_model;//.cast<double>();


        forces_counter += num_stance_feet;
        num_stance_feet_it[k] = num_stance_feet;


    }

    /*
    int min_col;
    int max_col = 0;
    for (int i=0; i < HORIZON; i++){
        max_col += num_stance_feet_it[i];
        min_col = max_col - num_stance_feet_it[i];
        std::cout << "num stance feet " << num_stance_feet_it[i] << " min max col, k = " << i << " [" << min_col << ", " << max_col << "]" << std::endl;

        for (int j=0; j < HORIZON; j++){
            if (j > i){
                Bqp.block(j*DYN_STATE_DIM, 3 * min_col, DYN_STATE_DIM, 3 * num_stance_feet_it[i] ) = A_consecutive.block((j-i)*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) * Bqp.block(i * DYN_STATE_DIM, 3 * min_col, DYN_STATE_DIM, 3 * num_stance_feet_it[i] );

            }
        }
    }
    */




    /****************** QP ***********************/


    Eigen::MatrixXd Q(forces_counter*3, forces_counter*3);
    Eigen::VectorXd q( 3* forces_counter);






    // Q = 2*( (Bqp.transpose().cast<double>() * L * Bqp.cast<double>())
    // +  Eigen::MatrixXd::Identity(3* forces_counter, 3* forces_counter) *this->w_forces);


    // q =  2* (Bqp.transpose().cast<double>()* L) * ( Aqp.cast<double>() * this->initial_state.cast<double>() - this->state_ref.cast<double>() );

    Q = 2*( (Bqp.transpose() * L * Bqp)
            +  Eigen::MatrixXd::Identity(3* forces_counter, 3* forces_counter) * this->w_forces);


    q =  2* (Bqp.transpose()* L)* ( Aqp * this->initial_state - this->state_ref );


    /*
    std::cout << "------------" << std::endl;
    for (int i=0; i < forces_counter*3; i++){
        for (int j=0; j < forces_counter*3; j++) {

            std::cout << Q(i,j) << ", ";
        }
        std::cout << std::endl;
    }
    */




    Eigen::VectorXd forces(3*forces_counter);

    if (USE_QPOASES == 0){
        // std::cout << "QuadProgPP" << std::endl;

        Eigen::MatrixXd Cd_comp(2*forces_counter*5, forces_counter*3);
        Eigen::VectorXd D_comp(2*forces_counter*5);


        Cd_comp << Cqp, -Cqp;
        D_comp  << -D_MINqp, D_MAXqp;




        // Empty equality equations
        Eigen::MatrixXd CE(3*forces_counter, 0);
        Eigen::VectorXd ce(0);



        QuadProgPP::solve_quadprog(Q,
                                   q,
                                   CE,
                                   ce,
                                   Cd_comp.transpose(), //
                                   D_comp,// D_MAXd,//
                                   forces);


    }
    /****************************************************/

    if (USE_QPOASES == 1){
        // std::cout << "QPOASES" << std::endl;
        Eigen::MatrixXd Cqp_double(forces_counter*5, forces_counter*3);
        Eigen::VectorXd D_MIN_double(forces_counter*5);
        Eigen::VectorXd D_MAX_double(forces_counter*5);

        Cqp_double   = Cqp;//-Cqp;
        D_MIN_double = D_MINqp;// - Cqp * NORMALS;
        D_MAX_double = D_MAXqp;// - Cqp * NORMALS;

        std::cout << "REEEEE" << std::endl;



        /*
        std::cout << "----Cqp------" << std::endl;
        for (int i=0; i < forces_counter*5; i++){
            for (int j=0; j < forces_counter*3; j++) {
                std::cout << Cqp_double(i,j) << ", ";
            }
            std::cout << std::endl << std::endl;
        }
        std::cout << "----Dmin------" << std::endl;
        for (int i=0; i < forces_counter*5; i++){
            std::cout << D_MIN_double(i) << ", ";
        }
        std::cout << std::endl;

        std::cout << "----Dmax------" << std::endl;
        for (int i=0; i < forces_counter*5; i++){
            std::cout << D_MAX_double(i) << ", ";
        }
        std::cout << std::endl;

        */
        // Cqp_double = -Cqp;
        // D_MIN_double = D_MINqp;
        // D_MAX_double = D_MAXqp;

        Eigen::VectorXd MIN_FORCE(forces_counter*3);
        Eigen::VectorXd MAX_FORCE(forces_counter*3);
        MIN_FORCE.fill(0);
        MAX_FORCE.fill(60);

        USING_NAMESPACE_QPOASES
        QProblem solve_qp(3*forces_counter, forces_counter*5);

        Options op;
        op.setToMPC();
        op.printLevel = qpOASES::PL_NONE;
        solve_qp.setOptions(op);

        const real_t* H = Q.data();
        const real_t* g = q.data();
        const real_t* A =   Cqp_double.data();
        const real_t* lbA = D_MIN_double.data();
        const real_t* ubA = D_MAX_double.data();
        const real_t* lb = NULL; //MIN_FORCE.data();
        const real_t* ub = NULL; //MAX_FORCE.data();


        int nWSR = 100;

        returnValue hr;
        real_t maxCPUtime = 0.003;

        hr = solve_qp.init(H,
                           g,
                           A,
                           lb,
                           ub,
                           lbA,
                           ubA,
                           nWSR,
                           &maxCPUtime);




        // std::cout << "RETURN VAL " << hr << std::endl;


        real_t xOpt[3*forces_counter];
        solve_qp.getPrimalSolution( xOpt );

        /*

        if(hr == 0){
            std::cout << "FORCES" << std::endl;

            for (int i=0; i <forces_counter; i++){
                std::cout << xOpt[3*i] << ", " << xOpt[3*i + 1] << ", " << xOpt[3*i + 2] << std::endl;
            }
            std::cout << "-----------------------" <<  std::endl;
        }

        */


        for (int i=0; i< 3*forces_counter; i++){
            forces(i) = xOpt[i];
        }


    }


    Eigen::MatrixXd computed_forces(3*forces_counter, 1);
    computed_forces << forces;
    Eigen::MatrixXd predicted_state(HORIZON * DYN_STATE_DIM, 1);
    predicted_state = Aqp * this->initial_state + Bqp * computed_forces;


    // std::cout << Bqp.rows() << "x" << Bqp.cols() << "    " << 3*forces_counter << std::endl;

    ref_base_vel     << this->state_ref(DYN_STATE_DIM*0+9),  this->state_ref(DYN_STATE_DIM*0 + 10), this->state_ref(DYN_STATE_DIM*0 + 11);

    if (ref_base_vel.norm() > 0){
        /*
        std::cout << "base_vel: " << ref_base_vel.x() << " " << ref_base_vel.y() << " " << ref_base_vel.z() << std::endl;

        std::cout << "---------- PREDICTION ---------------" << std::endl;

        for (int k=0; k < HORIZON; k++){
            ref_base_ang     << predicted_state(DYN_STATE_DIM*k),    predicted_state(DYN_STATE_DIM*k + 1),  predicted_state(DYN_STATE_DIM*k + 2);
            ref_base_pos     << predicted_state(DYN_STATE_DIM*k+3),  predicted_state(DYN_STATE_DIM*k + 4),  predicted_state(DYN_STATE_DIM*k + 5);
            ref_base_ang_vel << predicted_state(DYN_STATE_DIM*k+6),  predicted_state(DYN_STATE_DIM*k + 7),  predicted_state(DYN_STATE_DIM*k + 8);
            ref_base_vel     << predicted_state(DYN_STATE_DIM*k+9),  predicted_state(DYN_STATE_DIM*k + 10), predicted_state(DYN_STATE_DIM*k + 11);

            std::cout << k << ":" << std::endl;
            std::cout << "base_pos: " << ref_base_pos.x() << " " << ref_base_pos.y() << " " << ref_base_pos.z() << std::endl;
            std::cout << "base_vel: " << ref_base_vel.x() << " " << ref_base_vel.y() << " " << ref_base_vel.z() << std::endl;
            std::cout << "base_ang: " << ref_base_ang.x() << " " << ref_base_ang.y() << " " << ref_base_ang.z() << std::endl;
            std::cout << "ang_vel:  " << ref_base_ang_vel.x() << " " << ref_base_ang_vel.y() << " " << ref_base_ang_vel.z() << std::endl << std::endl;
        }
        std::cout << std::endl;

        std::cout << "-------------------------------" << std::endl;
        */
    }


    /*****************************************************/


    this->curr_forces.setZero();
    offset_feet = 0;
    if (this->stance_fl){
        this->curr_forces.operator()(0) = NORMALS(3*offset_feet + 0) - forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(1) = NORMALS(3*offset_feet + 1) - forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(2) = NORMALS(3*offset_feet + 2) - forces.operator()(3*offset_feet + 2);
        offset_feet += 1;
    }

    if(this->stance_fr){
        this->curr_forces.operator()(3) = NORMALS(3*offset_feet + 0) - forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(4) = NORMALS(3*offset_feet + 1) - forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(5) = NORMALS(3*offset_feet + 2) - forces.operator()(3*offset_feet + 2);
        offset_feet += 1;
    }

    if(this->stance_bl){
        this->curr_forces.operator()(6) = NORMALS(3*offset_feet + 0) - forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(7) = NORMALS(3*offset_feet + 1) - forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(8) = NORMALS(3*offset_feet + 2) - forces.operator()(3*offset_feet + 2);
        offset_feet += 1;
    }

    if(this->stance_br){
        this->curr_forces.operator()(9)  = NORMALS(3*offset_feet + 0) - forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(10) = NORMALS(3*offset_feet + 1) - forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(11) = NORMALS(3*offset_feet + 2) - forces.operator()(3*offset_feet + 2);
    }

    /*
    std::cout << "FORCES" << std::endl;

    for (int i=0; i <4; i++){
        std::cout << this->curr_forces(3*i) << ", " << this->curr_forces(3*i + 1) << ", " << this->curr_forces(3*i + 2) << std::endl;
    }
    std::cout << "-----------------------" <<  std::endl;
    */




}








/**************************** QPoases *******************************/
/*
void PredictiveModeController::computeForcesQPoases(double current_main_phase){
    Matrix13f    Adyn;
    Matrix13x12f Bdyn;

    Eigen::Matrix4f trans_body_matrix;

    Eigen::Vector3f ref_base_ang, ref_base_pos, ref_base_ang_vel, ref_base_vel;
    Eigen::Vector3f feet_pos_fl, feet_pos_fr, feet_pos_bl, feet_pos_br;
    Eigen::Vector3f feet_pos_fl_fromBase, feet_pos_fr_fromBase, feet_pos_bl_fromBase, feet_pos_br_fromBase;
    Eigen::Vector3f feet_pos_fl_fromWorld, feet_pos_fr_fromWorld, feet_pos_bl_fromWorld, feet_pos_br_fromWorld;
    Eigen::Vector3f pos_fl, pos_fr, pos_bl, pos_br;

    Eigen::Vector3f normal_force;

    bool stance_fl_, stance_fr_, stance_bl_, stance_br_;
    int num_stance_feet, num_stance_feet_k0, offset_feet;



    // Big matrices for QP optimisation
    Eigen::MatrixXf Aqp(HORIZON * DYN_STATE_DIM, DYN_STATE_DIM);
    Eigen::MatrixXf Bqp(HORIZON * DYN_STATE_DIM, HORIZON * 4 * 3);
    Eigen::MatrixXd Cqp(HORIZON * 4 * 5,  HORIZON * 4 * 3);
    Eigen::MatrixXd D_MINqp(HORIZON * 4 * 5, 1);
    Eigen::MatrixXd D_MAXqp(HORIZON * 4 * 5, 1);
    Eigen::MatrixXd L(HORIZON * DYN_STATE_DIM, HORIZON * DYN_STATE_DIM); // weights dynamical model
    Eigen::MatrixXd NORMALS(HORIZON * 4 * 3, 1);

    Eigen::MatrixXd STANCE_FEETS(HORIZON * 4 * 3, 1);

    L.setZero();

    Aqp.setZero();
    Bqp.setZero();
    Cqp.setZero();
    D_MINqp.setZero();
    D_MAXqp.setZero();
    NORMALS.setZero();
    STANCE_FEETS = Eigen::MatrixXd::Constant(HORIZON * 4 * 3, 1, 10);

    bool stance_feet[4];
    for(int k=0; k<HORIZON; k++) {
        ref_base_ang     << this->state_ref(DYN_STATE_DIM*k),    this->state_ref(DYN_STATE_DIM*k + 1),  this->state_ref(DYN_STATE_DIM*k + 2);
        ref_base_pos     << this->state_ref(DYN_STATE_DIM*k+3),  this->state_ref(DYN_STATE_DIM*k + 4),  this->state_ref(DYN_STATE_DIM*k + 5);
        ref_base_ang_vel << this->state_ref(DYN_STATE_DIM*k+6),  this->state_ref(DYN_STATE_DIM*k + 7),  this->state_ref(DYN_STATE_DIM*k + 8);
        ref_base_vel     << this->state_ref(DYN_STATE_DIM*k+9),  this->state_ref(DYN_STATE_DIM*k + 10), this->state_ref(DYN_STATE_DIM*k + 11);


        // std::cout << k << ": " << ref_base_pos.x() << " " << ref_base_pos.y() << " " << ref_base_pos.z() << std::endl;


        stance_fl_ = this->fl_trajectory->predictFeetState(current_main_phase, (k+1)*DELTAT, ref_base_vel, &feet_pos_fl);
        stance_fr_ = this->fr_trajectory->predictFeetState(current_main_phase, (k+1)*DELTAT, ref_base_vel, &feet_pos_fr);
        stance_bl_ = this->bl_trajectory->predictFeetState(current_main_phase, (k+1)*DELTAT, ref_base_vel, &feet_pos_bl);
        stance_br_ = this->br_trajectory->predictFeetState(current_main_phase, (k+1)*DELTAT, ref_base_vel, &feet_pos_br);

        stance_feet[0] = stance_fl_;
        stance_feet[1] = stance_fr_;
        stance_feet[2] = stance_bl_;
        stance_feet[3] = stance_br_;

        this->kinematics->getFeetFromBody("fl", feet_pos_fl, &feet_pos_fl_fromBase);
        this->kinematics->getFeetFromBody("fr", feet_pos_fr, &feet_pos_fr_fromBase);
        this->kinematics->getFeetFromBody("bl", feet_pos_bl, &feet_pos_bl_fromBase);
        this->kinematics->getFeetFromBody("br", feet_pos_br, &feet_pos_br_fromBase);

        this->kinematics->getTransform(ref_base_pos, ref_base_ang, &trans_body_matrix);

        this->kinematics->applyTransform(feet_pos_fl_fromBase,  trans_body_matrix, &feet_pos_fl_fromWorld);
        this->kinematics->applyTransform(feet_pos_fr_fromBase,  trans_body_matrix, &feet_pos_fr_fromWorld);
        this->kinematics->applyTransform(feet_pos_bl_fromBase,  trans_body_matrix, &feet_pos_bl_fromWorld);
        this->kinematics->applyTransform(feet_pos_br_fromBase,  trans_body_matrix, &feet_pos_br_fromWorld);

        feet_pos_fl_fromWorld.z() = 0;
        feet_pos_fr_fromWorld.z() = 0;
        feet_pos_bl_fromWorld.z() = 0;
        feet_pos_br_fromWorld.z() = 0;


        pos_fl = ref_base_pos - feet_pos_fl_fromWorld;
        pos_fr = ref_base_pos - feet_pos_fr_fromWorld;
        pos_bl = ref_base_pos - feet_pos_bl_fromWorld;
        pos_br = ref_base_pos - feet_pos_br_fromWorld;

        // feet_pos_fl_fromBase(2) = 0;
        // feet_pos_fr_fromBase(2) = 0;
        // feet_pos_bl_fromBase(2) = 0;
        // feet_pos_br_fromBase(2) = 0;


        // std::cout << pos_fl.x() << " " << pos_fl.y() << " " << pos_fl.z() << std::endl;
        // std::cout << feet_pos_fr_fromBase.x() << " " << feet_pos_fr_fromBase.y() << " " << feet_pos_fr_fromBase.z() << std::endl;
        // std::cout << feet_pos_bl_fromBase.x() << " " << feet_pos_bl_fromBase.y() << " " << feet_pos_bl_fromBase.z() << std::endl;
        // std::cout << feet_pos_br_fromBase.x() << " " << feet_pos_br_fromBase.y() << " " << feet_pos_br_fromBase.z() << std::endl;


        num_stance_feet = stance_fl_ + stance_fr_ + stance_bl_ + stance_br_;
        if (k == 0){
            num_stance_feet_k0 = num_stance_feet;
            this->stance_fl = stance_fl_;
            this->stance_fr = stance_fr_;
            this->stance_bl = stance_bl_;
            this->stance_br = stance_br_;
        }

        // std::cout << "stance feets " << num_stance_feet << std::endl;

        float yaw = ref_base_ang.z();

        Eigen::Matrix3f Rz, InertiaTrans, InertiaInv;
        this->getRz(yaw, &Rz);
        InertiaTrans = Rz * this->Inertia * Rz.transpose();
        InertiaInv = InertiaTrans.inverse();


        this->getAdynMatrix(yaw, DELTAT,
                            InertiaInv,
                            pos_fl,
                            pos_fr,
                            pos_bl,
                            pos_br,
                            stance_feet,
                            &Adyn);


        this->getBdynMatrix(yaw, DELTAT,
                            InertiaInv,
                            pos_fl,
                            pos_fr,
                            pos_bl,
                            pos_br,
                            &Bdyn);

        // this->getAdynAndBdynMatrices(DELTAT, A, B, &Adyn, &Bdyn);


        Eigen::MatrixXf D_MIN(4*5, 1);
        Eigen::MatrixXf D_MAX(4*5, 1);
        Eigen::MatrixXf C(4*5, 4*3);
        C.setZero();


        C.block(0, 0, 5, 3)    = this->terrain_fl;
        D_MIN.block(0,0, 5, 1) = this->d_min;
        D_MAX.block(0,0, 5, 1) = this->d_max;

        C.block(5, 3, 5, 3)    = this->terrain_fr;
        D_MIN.block(5,0, 5, 1) = this->d_min;
        D_MAX.block(5,0, 5, 1) = this->d_max;

        C.block(5*2, 3*2, 5, 3)  = this->terrain_bl;
        D_MIN.block(5*2,0, 5, 1) = this->d_min;
        D_MAX.block(5*2,0, 5, 1) = this->d_max;

        C.block(5*3, 3*3, 5, 3)  = this->terrain_br;
        D_MIN.block(5*3,0, 5, 1) = this->d_min;
        D_MAX.block(5*3,0, 5, 1) = this->d_max;




        if (k ==0) Aqp.block(0, 0, DYN_STATE_DIM, DYN_STATE_DIM) = Adyn;
        else Aqp.block(k*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) = Aqp.block((k-1)*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) * Adyn;



        for (int j=0; j < HORIZON; j ++){

            if(j == k){
                Bqp.block(j*DYN_STATE_DIM, k * 4 * 3, DYN_STATE_DIM, 3 * 4) = Bdyn;
            }
            if (j > k){
                // Bqp.block(j*DYN_STATE_DIM, 3 * forces_counter, DYN_STATE_DIM, 3 * num_stance_feet) = Adyn.pow(j-k) * Bdyn_red;
                Bqp.block(j*DYN_STATE_DIM, k * 4 * 3, DYN_STATE_DIM, 3 * 4) = Aqp.block((j-k-1)*DYN_STATE_DIM, 0, DYN_STATE_DIM, DYN_STATE_DIM) * Bdyn;
            }
        }


        Cqp.block( k*4*5, k*4*3, 4*5, 4*3) = C.cast<double>();
        D_MINqp.block( k*4*5, 0, 4*5, 1)   = D_MIN.cast<double>();
        D_MAXqp.block( k*4*5, 0, 4*5, 1)   = D_MAX.cast<double>();

        L.block(k*DYN_STATE_DIM, k*DYN_STATE_DIM, DYN_STATE_DIM, DYN_STATE_DIM) = this->w_dyn_model.cast<double>();


        normal_force << 0, 0, (ROBOT_TOTAL_MASS/num_stance_feet)*9.8;
        for(int i=0; i<num_stance_feet; i++){
            NORMALS.block(k*4*3, 0, 3, 1) = normal_force.cast<double>();
        }

        if(stance_fl_) STANCE_FEETS.block(k*4*3, 0, 3, 1).setZero();
        if(stance_fr_) STANCE_FEETS.block(k*4*3+3, 0, 3, 1).setZero();
        if(stance_bl_) STANCE_FEETS.block(k*4*3+6, 0, 3, 1).setZero();
        if(stance_br_) STANCE_FEETS.block(k*4*3+9, 0, 3, 1).setZero();

    }


    // for (int i=0; i <HORIZON*4*3; i++){
    // std::cout << STANCE_FEETS(i) << ", ";
    // }
    // std::cout << std::endl;

    Eigen::MatrixXd Q(HORIZON*4*3, HORIZON*4*3);
    Eigen::VectorXd q(HORIZON*4*3);

    Q = 2*( (Bqp.transpose().cast<double>() * L * Bqp.cast<double>())
            +  Eigen::MatrixXd::Identity(HORIZON*4*3, HORIZON*4*3) *this->w_forces);


    q =  2* (Bqp.transpose().cast<double>()* L ) * ( Aqp.cast<double>() * this->initial_state.cast<double>() - this->state_ref.cast<double>() );





    Eigen::MatrixXd Cqp_double(HORIZON*4*5, HORIZON*4*3);
    Eigen::VectorXd D_MIN_double(HORIZON*4*5);
    Eigen::VectorXd D_MAX_double(HORIZON*4*5);
    D_MIN_double = D_MINqp - Cqp * NORMALS;
    D_MAX_double = D_MAXqp - Cqp * NORMALS;
    Cqp_double = -Cqp;

    Eigen::MatrixXd L_lim(HORIZON*4*3, 1);
    L_lim = -STANCE_FEETS;


    const real_t* H = Q.data();
    const real_t* g = q.data();
    const real_t* K =   Cqp_double.data();
    const real_t* lbK = D_MIN_double.data();
    const real_t* ubK = D_MAX_double.data();
    const real_t* lb = L_lim.data(); //MIN_FORCE.data();
    const real_t* ub = STANCE_FEETS.data(); //MAX_FORCE.data();

    int nWSR = 1000;

    returnValue hr;

    if (!this->init_qp){
        hr = this->solve_qp->init(H,
                                   g,
                                   K,
                                   lb,
                                   ub,
                                   lbK,
                                   ubK,
                                   nWSR);
        this->init_qp = true;
    }

    else{
        hr = this->solve_qp->hotstart(H,
                                       g,
                                       K,
                                       lb,
                                       ub,
                                       lbK,
                                       ubK,
                                       nWSR);
    }


    std::cout << "RETURN VAL " << hr << std::endl;


    real_t xOpt[HORIZON*4*3];
    this->solve_qp->getPrimalSolution( xOpt );


    // real_t yOpt[HORIZON*4*3 + HORIZON*4*5];
    // solve_qp.getDualSolution( yOpt );


    this->curr_forces.setZero();
    if(hr == 0){
        // std::cout << "FORCES" << std::endl;
        for (int i=0; i <12; i++){
            // std::cout << xOpt[3*i] << ", " << xOpt[3*i + 1] << ", " << xOpt[3*i + 2] << std::endl;
            this->curr_forces(i) = xOpt[i];


        }
        // std::cout << "-----------------------" <<  std::endl;

    }
}

*/








void PredictiveModeController::getForces(Eigen::Vector3d* force_fl, Eigen::Vector3d* force_fr, Eigen::Vector3d* force_bl, Eigen::Vector3d* force_br){
	int offset = 0;
    force_fl->operator()(0) = this->curr_forces.operator()(0 + offset) * DELTAT;
    force_fl->operator()(1) = this->curr_forces.operator()(1 + offset) * DELTAT;
    force_fl->operator()(2) = this->curr_forces.operator()(2 + offset) * DELTAT;

    force_fr->operator()(0) = this->curr_forces.operator()(3+ offset) * DELTAT;
    force_fr->operator()(1) = this->curr_forces.operator()(4+ offset) * DELTAT;
    force_fr->operator()(2) = this->curr_forces.operator()(5+ offset) * DELTAT;

    force_bl->operator()(0) = this->curr_forces.operator()(6+ offset) * DELTAT;
    force_bl->operator()(1) = this->curr_forces.operator()(7+ offset) * DELTAT;
    force_bl->operator()(2) = this->curr_forces.operator()(8+ offset) * DELTAT;

    force_br->operator()(0) = this->curr_forces.operator()(9 + offset) * DELTAT;
    force_br->operator()(1) = this->curr_forces.operator()(10+ offset) * DELTAT;
    force_br->operator()(2) = this->curr_forces.operator()(11+ offset) * DELTAT;
}
