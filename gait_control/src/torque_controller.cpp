//
// Created by andres on 07.11.20.
//


#include "../include/torque_controller.h"


torqueController::torqueController(Kinematics* kinematics){
    this->kinematics = kinematics;

    this->feet_pos_fl << 0,0,0;
    this->feet_pos_fr << 0,0,0;
    this->feet_pos_bl << 0,0,0;
    this->feet_pos_br << 0,0,0;

    this->stance_fl = true;
    this->stance_fr = true;
    this->stance_bl = true;
    this->stance_br = true;

    this->terrain_fl =  Eigen::MatrixXf::Zero(5, 3);
    this->terrain_fr =  Eigen::MatrixXf::Zero(5, 3);
    this->terrain_bl =  Eigen::MatrixXf::Zero(5, 3);
    this->terrain_br =  Eigen::MatrixXf::Zero(5, 3);

    this->d_min << 0,0,0,0,0;
    this->d_max << 0,0,0,0,0;

    this->num_stand_feets = 4;

    this->curr_forces.resize(this->num_stand_feets*3);
    this->curr_forces.setZero();

    this->prev_forces.resize(this->num_stand_feets*3);
    this->prev_forces.setZero();

    this->k_pos.setZero();
    this->k_vel.setZero();
    this->k_rot.setZero();
    this->k_ang_vel.setZero();

    this->w_dyn_model   = 1;
    this->w_forces      = 0.5;
    this->w_prev_forces = 0.5;

    this->Inertia << ROBOT_INERTIA_XX, ROBOT_INERTIA_XY, ROBOT_INERTIA_XZ,
                     ROBOT_INERTIA_XY, ROBOT_INERTIA_YY, ROBOT_INERTIA_YZ,
                     ROBOT_INERTIA_XZ, ROBOT_INERTIA_YZ, ROBOT_INERTIA_ZZ;

    this->base_pos_offset.setZero();

}


torqueController::~torqueController(){

}


void torqueController::setBaseCoMOffset(float base_offset_x, float base_offset_y){
    this->base_pos_offset << base_offset_x, base_offset_y, 0;
}


void torqueController::setRobotState(const Eigen::Vector3f &base_pos,     const Eigen::Vector3f &base_vel,     const Eigen::Matrix3f &base_rot, const Eigen::Vector3f &base_ang_vel){
    this->base_pos.operator=(     base_pos );
    this->base_vel.operator=(     base_vel );
    this->base_rot.operator=(     base_rot );
    this->base_ang_vel.operator=( base_ang_vel );


    // std::cout << "state: " << base_pos.x() << " " << base_pos.y() << " " << base_pos.z() << std::endl;

}



void torqueController::setRobotControl(const Eigen::Vector3f &des_base_pos, const Eigen::Vector3f &des_base_vel, const Eigen::Matrix3f &des_base_rot, const Eigen::Vector3f &des_base_ang_vel){
    this->des_base_pos.operator=(     des_base_pos );
    this->des_base_vel.operator=(     des_base_vel );
    this->des_base_rot.operator=(     des_base_rot );
    this->des_base_ang_vel.operator=( des_base_ang_vel );
    // std::cout << "desired: " << des_base_pos.x() << " " << des_base_pos.y() << " " << des_base_pos.z() << std::endl << std::endl;

}




void torqueController::setLegState(std::string leg, const Eigen::Vector3f &feet_pos, bool stance_state){


    if (leg == "fl"){
        this->feet_pos_fl.operator=(feet_pos);
        this->stance_fl = stance_state;
    }

    if (leg == "fr"){
        this->feet_pos_fr.operator=(feet_pos);
        this->stance_fr = stance_state;
    }

    if (leg == "bl"){
        this->feet_pos_bl.operator=(feet_pos);
        this->stance_bl = stance_state;
    }

    if (leg == "br"){
        this->feet_pos_br.operator=(feet_pos);
        this->stance_br = stance_state;
    }
}


void torqueController::setTerrainPlane(std::string leg, float mu, const Eigen::Vector3f &normal, const Eigen::Vector3f &tangent1, const Eigen::Vector3f &tangent2){
    Eigen::Matrix<float,5,3> terrain_mat;
    terrain_mat << -mu * normal.x() + tangent1.x(),  -mu * normal.y() + tangent1.y(), -mu * normal.z() + tangent1.z(),
                   -mu * normal.x() + tangent2.x(),  -mu * normal.y() + tangent2.y(), -mu * normal.z() + tangent2.z(),
                    mu * normal.x() + tangent2.x(),   mu * normal.y() + tangent2.y(),  mu * normal.z() + tangent2.z(),
                    mu * normal.x() + tangent1.x(),   mu * normal.y() + tangent1.y(),  mu * normal.z() + tangent1.z(),
                    normal.x(),                       normal.y(),                      normal.z();


    // TO-DO
    terrain_mat << 0,0,0,
                   0,0,0,
                   0,0,0,
                   0,0,0,
                   0,0,0;


    if (leg == "fl") this->terrain_fl.operator=(terrain_mat);
    if (leg == "fr") this->terrain_fr.operator=(terrain_mat);
    if (leg == "bl") this->terrain_bl.operator=(terrain_mat);
    if (leg == "br") this->terrain_br.operator=(terrain_mat);

}


void torqueController::setFeetForceLimits(float min_force, float max_force){

    this->d_min << - 1000000000000,  // large negative number, in theory - inf
                   - 1000000000000,  // large negative number, in theory - inf
                     0,
                     0,
                     min_force;

    this->d_max <<   0,
                     0,
                     1000000000000,  // large positive number, in theory  inf
                     1000000000000,  // large positive number, in theory  inf
                     max_force;


}


void torqueController::setProportionalConstController(Eigen::Vector3f k_pos, Eigen::Vector3f k_vel, Eigen::Vector3f k_rot, Eigen::Vector3f k_ang_vel){
    this->k_pos << k_pos.x(), 0, 0,
                   0, k_pos.y(), 0,
                   0, 0, k_pos.z();

    this->k_vel << k_vel.x(), 0, 0,
                   0, k_vel.y(), 0,
                   0, 0, k_vel.z();

    this->k_rot << k_rot.x(), 0, 0,
                   0, k_rot.y(), 0,
                   0, 0, k_rot.z();

    this->k_ang_vel << k_ang_vel.x(), 0, 0,
                   0, k_ang_vel.y(), 0,
                   0, 0, k_ang_vel.z();
}

void torqueController::setWeightsController(float w_dyn_model, float w_forces, float w_prev_forces){
    this->w_dyn_model = w_dyn_model;
    this->w_forces    = w_forces;
    this->w_prev_forces = w_prev_forces;
}



void torqueController::getVectorialMulMatrix(const Eigen::Vector3f &v, Eigen::Matrix3f* m){
    m->operator()(0,0) =  0;
    m->operator()(0,1) = -v.z();
    m->operator()(0,2) =  v.y();

    m->operator()(1,0) =  v.z();
    m->operator()(1,1) =  0;
    m->operator()(1,2) = -v.x();

    m->operator()(2,0) = -v.y();
    m->operator()(2,1) =  v.x();
    m->operator()(2,2) =  0;
}

int torqueController::countStanceFeets(){
    int num_stance_feets = 0;
    num_stance_feets += this->stance_fl;
    num_stance_feets += this->stance_fr;
    num_stance_feets += this->stance_bl;
    num_stance_feets += this->stance_br;
    return num_stance_feets;
}



void torqueController::computeForces(double dt){

    // typedef Eigen::Matrix<Scalar,-1,-1> MatrixXs;
    // typedef Eigen::Matrix<Scalar,-1,1> VectorXs;

    int num_feets = this->countStanceFeets();



    Eigen::MatrixXf A(6, 3*num_feets);

    Eigen::VectorXf D_MIN(num_feets*5);
    Eigen::VectorXf D_MAX(num_feets*5);
    Eigen::MatrixXf C(num_feets*5, num_feets*3);
    Eigen::VectorXf NORMALS(3*num_feets);

    C.setZero();

    Eigen::VectorXf prev_computed_forces(3*num_feets);

    Eigen::Matrix4f trans_body_matrix;
    // this->kinematics->getTransform(this->base_pos, this->base_rot, &trans_body_matrix);

    /*
    Eigen::Vector3f feet_pos_fl_fromBase, feet_pos_fr_fromBase, feet_pos_bl_fromBase, feet_pos_br_fromBase;

    this->kinematics->getFeetFromBody("fl", this->feet_pos_fl, &feet_pos_fl_fromBase);
    this->kinematics->getFeetFromBody("fr", this->feet_pos_fr, &feet_pos_fr_fromBase);
    this->kinematics->getFeetFromBody("bl", this->feet_pos_bl, &feet_pos_bl_fromBase);
    this->kinematics->getFeetFromBody("br", this->feet_pos_br, &feet_pos_br_fromBase);

    Eigen::Vector3f feet_pos_fl_fromWorld, feet_pos_fr_fromWorld, feet_pos_bl_fromWorld, feet_pos_br_fromWorld;

    this->kinematics->applyTransform(feet_pos_fl_fromBase,  trans_body_matrix, &feet_pos_fl_fromWorld);
    this->kinematics->applyTransform(feet_pos_fr_fromBase,  trans_body_matrix, &feet_pos_fr_fromWorld);
    this->kinematics->applyTransform(feet_pos_bl_fromBase,  trans_body_matrix, &feet_pos_bl_fromWorld);
    this->kinematics->applyTransform(feet_pos_br_fromBase,  trans_body_matrix, &feet_pos_br_fromWorld);

    feet_pos_fl_fromWorld.z() = 0;
    feet_pos_fr_fromWorld.z() = 0;
    feet_pos_bl_fromWorld.z() = 0;
    feet_pos_br_fromWorld.z() = 0;

    Eigen::Vector3f pos_fl, pos_fr, pos_bl, pos_br;

    this->feet_pos_fl.z() = 0;
    this->feet_pos_fr.z() = 0;
    this->feet_pos_bl.z() = 0;
    this->feet_pos_br.z() = 0;

    pos_fl = feet_pos_fl_fromWorld - this->base_pos + base_CoM_offset;
    pos_fr = feet_pos_fr_fromWorld - this->base_pos + base_CoM_offset;
    pos_bl = feet_pos_bl_fromWorld - this->base_pos + base_CoM_offset;
    pos_br = feet_pos_br_fromWorld - this->base_pos + base_CoM_offset;


    // pos_fl = this->feet_pos_fl - this->base_pos;// + base_CoM_offset;
    // pos_fr = this->feet_pos_fr - this->base_pos;// + base_CoM_offset;
    // pos_bl = this->feet_pos_bl - this->base_pos;// + base_CoM_offset;
    // pos_br = this->feet_pos_br - this->base_pos;// + base_CoM_offset;
    */
    Eigen::Vector3f pos_fl, pos_fr, pos_bl, pos_br;

    pos_fl = this->feet_pos_fl - this->base_pos + base_pos_offset;
    pos_fr = this->feet_pos_fr - this->base_pos + base_pos_offset;
    pos_bl = this->feet_pos_bl - this->base_pos + base_pos_offset;
    pos_br = this->feet_pos_br - this->base_pos + base_pos_offset;

    // std::cout << pos_fl.x() << " " << pos_fl.y() << " " << pos_fl.z() << std::endl;

    Eigen::Vector3f acc_gravity_contribution, gravity, normal_vect;
    acc_gravity_contribution << 0,0,0;
    gravity << 0, 0, -9.8;
    normal_vect << 0, 0, 9.8/num_feets;
    int offset_feet = 0;
    if(this->stance_fl){
        Eigen::Matrix3f vect_mat_fl;
        this->getVectorialMulMatrix(pos_fl, &vect_mat_fl);
        A.block<3,3>(0,3*offset_feet) = -Eigen::MatrixXf::Identity(3, 3)/num_feets;
        A.block<3,3>(3,3*offset_feet) = -vect_mat_fl/num_feets;
        acc_gravity_contribution = acc_gravity_contribution + vect_mat_fl * normal_vect;

        C.block<5,3>(5*offset_feet, 3*offset_feet) = this->terrain_fl;

        prev_computed_forces.operator()(3*offset_feet + 0) = this->prev_forces.operator()(0);
        prev_computed_forces.operator()(3*offset_feet + 1) = this->prev_forces.operator()(1);
        prev_computed_forces.operator()(3*offset_feet + 2) = this->prev_forces.operator()(2);

        offset_feet += 1;
    }


    if(this->stance_fr){
        Eigen::Matrix3f vect_mat_fr;
        this->getVectorialMulMatrix(pos_fr, &vect_mat_fr);
        A.block<3,3>(0,3*offset_feet) = -Eigen::MatrixXf::Identity(3, 3)/num_feets;
        A.block<3,3>(3,3*offset_feet) = -vect_mat_fr/num_feets;
        acc_gravity_contribution = acc_gravity_contribution + vect_mat_fr * normal_vect;

        C.block<5,3>(5*offset_feet, 3*offset_feet) = this->terrain_fr;

        prev_computed_forces.operator()(3*offset_feet + 0) = this->prev_forces.operator()(3);
        prev_computed_forces.operator()(3*offset_feet + 1) = this->prev_forces.operator()(4);
        prev_computed_forces.operator()(3*offset_feet + 2) = this->prev_forces.operator()(5);

        offset_feet += 1;
    }


    if(this->stance_bl){
        Eigen::Matrix3f vect_mat_bl;
        this->getVectorialMulMatrix(pos_bl, &vect_mat_bl);
        A.block<3,3>(0,3*offset_feet) = -Eigen::MatrixXf::Identity(3, 3)/num_feets;
        A.block<3,3>(3,3*offset_feet) = -vect_mat_bl/num_feets;
        acc_gravity_contribution = acc_gravity_contribution + vect_mat_bl * normal_vect;

        C.block<5,3>(5*offset_feet, 3*offset_feet) = this->terrain_bl;

        prev_computed_forces.operator()(3*offset_feet + 0) = this->prev_forces.operator()(6);
        prev_computed_forces.operator()(3*offset_feet + 1) = this->prev_forces.operator()(7);
        prev_computed_forces.operator()(3*offset_feet + 2) = this->prev_forces.operator()(8);

        offset_feet += 1;
    }


    if(this->stance_br){
        Eigen::Matrix3f vect_mat_br;
        this->getVectorialMulMatrix(pos_br, &vect_mat_br);
        A.block<3,3>(0,3*offset_feet) = -Eigen::MatrixXf::Identity(3, 3)/num_feets;
        A.block<3,3>(3,3*offset_feet) = -vect_mat_br/num_feets;
        acc_gravity_contribution = acc_gravity_contribution + vect_mat_br * normal_vect;

        C.block<5,3>(5*offset_feet, 3*offset_feet) = this->terrain_br;

        prev_computed_forces.operator()(3*offset_feet + 0) = this->prev_forces.operator()(9);
        prev_computed_forces.operator()(3*offset_feet + 1) = this->prev_forces.operator()(10);
        prev_computed_forces.operator()(3*offset_feet + 2) = this->prev_forces.operator()(11);

    }

    Eigen::Vector3f normal_force;
    normal_force << 0, 0, (ROBOT_TOTAL_MASS/num_feets)*9.8;



    if(num_feets == 1){
        D_MIN.operator=(this->d_min);
        D_MAX.operator=(this->d_max);
        NORMALS.operator=(normal_force);
    }
    if(num_feets == 2){
        D_MIN << this->d_min, this->d_min;
        D_MAX << this->d_max, this->d_max;
        NORMALS << normal_force, normal_force;

    }
    if(num_feets == 3){
        D_MIN << this->d_min, this->d_min, this->d_min;
        D_MAX << this->d_max, this->d_max, this->d_max;
        NORMALS << normal_force, normal_force, normal_force;

    }
    if(num_feets == 4){
        D_MIN << this->d_min, this->d_min, this->d_min, this->d_min;
        D_MAX << this->d_max, this->d_max, this->d_max, this->d_max;
        NORMALS << normal_force, normal_force, normal_force, normal_force;

    }




    Eigen::Vector3f des_acc, des_ang_vel;


    // std::cout << "cur pos: " << this->base_pos.x() << " " << this->base_pos.y() << " " << this->base_pos.z() << std::endl;
    // std::cout << "des pos: " << this->des_base_pos.x() << " " << this->des_base_pos.y() << " " << this->des_base_pos.z() << std::endl << std::endl;



    des_acc.operator=(   this->k_pos*(this->des_base_pos - this->base_pos) +
                         this->k_vel*(this->des_base_vel - this->base_vel)  );


    tf::Matrix3x3 rot_diff_tf;
    Eigen::Matrix3d diff_rot = (this->des_base_rot * this->base_rot.transpose()).cast <double> ();   // float matrix to double matrix.

    tf::matrixEigenToTF(diff_rot, rot_diff_tf);
    double yaw, pitch, roll;
    rot_diff_tf.getEulerYPR(yaw, pitch, roll);
    Eigen::Vector3f dif_rot_ang;
    dif_rot_ang << roll, pitch, yaw;
    // dif_rot_ang(2) = 0;
    // std::cout << "dif angles" << roll << " " << pitch << " " << yaw << " " << std::endl;


    des_ang_vel.operator=(   this->k_rot * dif_rot_ang +
                             this->k_ang_vel*(this->des_base_ang_vel -  this->base_ang_vel)  );


    // std::cout << des_acc.x() << " " << des_acc.y() << " " << des_acc.z() << " " << std::endl;
    // std::cout << des_ang_vel.x() << " " << des_ang_vel.y() << " "<<  des_ang_vel.z() << " " << std::endl << std::endl;


    Eigen::Vector3f rot_dynamics;
    // rot_dynamics = Inertia * des_ang_vel;
    rot_dynamics = des_ang_vel;

    // if (num_feets < 4){
    // rot_dynamics = rot_dynamics  - (ROBOT_TOTAL_MASS/num_feets) * acc_gravity_contribution;
    // }


    Eigen::MatrixXf B(6, 1);
    B << des_acc.x(),
         des_acc.y(),
         des_acc.z(),
         rot_dynamics.x(),
         rot_dynamics.y(),
         rot_dynamics.z();



    /************************ QUADRATIC PROGRAMMING OPTIMISATION ****************************/


    Eigen::MatrixXf Q(3*num_feets, 3*num_feets);
    Eigen::VectorXf q( 3*num_feets);

    Q.operator=(   (A.transpose() * A).operator*(this->w_dyn_model) +
                    Eigen::MatrixXf::Identity(3*num_feets, 3*num_feets).operator*(this->w_forces) +
                    Eigen::MatrixXf::Identity(3*num_feets, 3*num_feets).operator*(this->w_prev_forces));

    q.operator=(   - A.transpose().operator*(this->w_dyn_model) * B
                   - prev_computed_forces.operator*(  this->w_prev_forces)  );

    // Empty equality equations
    Eigen::MatrixXd CE(3*num_feets,0);
    Eigen::VectorXd ce(0);


    Eigen::VectorXd forces(3*num_feets);


    Eigen::MatrixXd Qd(3*num_feets, 3*num_feets);
    Eigen::VectorXd qd( 3*num_feets);
    Eigen::VectorXd D_MINd(num_feets*5);
    Eigen::VectorXd D_MAXd(num_feets*5);
    Eigen::MatrixXd Cd(num_feets*5, num_feets*3);




    Qd = Q.cast <double> ();
    qd  = q.cast <double> ();
    Cd = C.cast <double> ();
    D_MINd = D_MIN.cast <double> ();
    D_MAXd = D_MAX.cast <double> ();

    Eigen::MatrixXd Cd_comp(2*num_feets*5, num_feets*3);
    Eigen::VectorXd D_comp(2*num_feets*5);

    Cd_comp << Cd, -Cd;
    D_comp  << -D_MINd + Cd * NORMALS.cast<double>(), D_MAXd - Cd * NORMALS.cast<double>();

    QuadProgPP::solve_quadprog(Qd,
                               qd,
                               CE,
                               ce,
                               Cd_comp.transpose(), // -Cd.transpose()
                               D_comp,// D_MAXd,//
                               forces);


    this->num_stand_feets = num_feets;




    // Set current forces

    offset_feet = 0;
    this->prev_forces.operator=( this->curr_forces );
    this->curr_forces.setZero();

    if (this->stance_fl){
        this->curr_forces.operator()(0) = forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(1) = forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(2) = forces.operator()(3*offset_feet + 2);
        offset_feet += 1;
    }

    if(this->stance_fr){
        this->curr_forces.operator()(3) = forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(4) = forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(5) = forces.operator()(3*offset_feet + 2);
        offset_feet += 1;
    }

    if(this->stance_bl){
        this->curr_forces.operator()(6) = forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(7) = forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(8) = forces.operator()(3*offset_feet + 2);
        offset_feet += 1;
    }

    if(this->stance_br){
        this->curr_forces.operator()(9)  = forces.operator()(3*offset_feet + 0);
        this->curr_forces.operator()(10) = forces.operator()(3*offset_feet + 1);
        this->curr_forces.operator()(11) = forces.operator()(3*offset_feet + 2);
    }


    /*
    std::cout  << "FORCES ";
    for(int i = 0; i < 12; i++){
        std::cout << this->curr_forces.operator()(i) << " ";
    }
    std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
    */












    /*

    // stupid example
    Eigen::MatrixXd Q(3,3);
    Q << 1,0,0,
         0,1,0,
         0,0,1;

    Eigen::VectorXd c(3);
    c << 0,0,0;

    Eigen::MatrixXd CI(3,3);
    CI << 1,0,0,
          0,1,0,
          0,0,1;

    Eigen::VectorXd ci(3);
    ci << -20,-20,-20;



    Eigen::VectorXd x(3);

    // qp solve
    // min (    1/2 x^T Q x + x^T q  )
    // subject to:  CI x + ci >= 0
    //

    Eigen::MatrixXd CE(3,0);
    Eigen::VectorXd ce(0);
    QuadProgPP::solve_quadprog(Q,c,CE,ce,CI,ci,x);

    for(int i = 0; i < 3; i++){
        std::cout << x.operator()(i) << " ";
    }
    std::cout << std::endl;
    */


}




void torqueController::getForces(Eigen::Vector3f* force_fl, Eigen::Vector3f* force_fr, Eigen::Vector3f* force_bl, Eigen::Vector3f* force_br){
    force_fl->operator()(0) = this->curr_forces.operator()(0);
    force_fl->operator()(1) = this->curr_forces.operator()(1);
    force_fl->operator()(2) = this->curr_forces.operator()(2);

    force_fr->operator()(0) = this->curr_forces.operator()(3);
    force_fr->operator()(1) = this->curr_forces.operator()(4);
    force_fr->operator()(2) = this->curr_forces.operator()(5);

    force_bl->operator()(0) = this->curr_forces.operator()(6);
    force_bl->operator()(1) = this->curr_forces.operator()(7);
    force_bl->operator()(2) = this->curr_forces.operator()(8);

    force_br->operator()(0) = this->curr_forces.operator()(9);
    force_br->operator()(1) = this->curr_forces.operator()(10);
    force_br->operator()(2) = this->curr_forces.operator()(11);
}


void torqueController::getKvel(Eigen::Vector3f * k_vel_vector){

    k_vel_vector->operator()(0) = this->k_vel(0,0);
    k_vel_vector->operator()(1) = this->k_vel(1,1);
    k_vel_vector->operator()(2) = this->k_vel(2,2);

}


