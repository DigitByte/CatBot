//
// Created by andres on 13.10.20.
//

#include "../include/feet_trajectory.h"
FeetTrajectory::FeetTrajectory(std::string name){

    this->stop_request_state = NON_STOP_REQUEST;
    this->walking           = true;


    this->leg_name = name;
    this->phase_offset = 0;
    this->leg_phase    = 0;
    this->feet_state_phase = 0;
    feet_state      = STANCE;
    feet_prev_state = STANCE;

    this->initial_pos << 0,0,0;
    this->current_pos << 0,0,0;
    this->current_pos_tmp << 0,0,0;

    this->middle_pos  << 0,0,0;
    this->final_pos   << 0,0,0;
    this->feet_pos_offset << 0,0,0;
    this->prev_feet_vel << 0,0,0;
    this->height = 0;
    this->set_initial_pos = false;

    this->max_leg_displacement = 0.1;

    this->integrated_feet_pos << 0, 0, 0;

    timer = ros::Time::now();

    this->use_integrated_pos = false;
    this->early_contact = false;

    this->prev_contact = 1;
    this->min_prop_swing_time_before_contact_detection = 1;
    this->dt_contact_detection_seconds = 0.5;
    this->capure_point_weight = 0;

}


FeetTrajectory::~FeetTrajectory(){

}

void FeetTrajectory::setKinematics(Kinematics* kinematics){
    this->kinematics = kinematics;
}

void FeetTrajectory::setPeriodAndSwing(double period, double prop_time_swing){
    this->period          = period;
    this->prop_time_swing = prop_time_swing;
}


void FeetTrajectory::setTrajectoryParams(double phase_offset, float height,  float feet_offset_x, float feet_offset_y, float base_offset_x, float base_offset_y, float capure_point_weight){
    this->phase_offset    = phase_offset;
    this->height          = height;
    this->capure_point_weight = capure_point_weight;
    this->feet_pos_offset << feet_offset_x, feet_offset_y, 0;
    this->base_pos_offset << base_offset_x, base_offset_y, 0;
    if ((this->leg_name == "fr")||(this->leg_name == "br")) this->feet_pos_offset.y() = -this->feet_pos_offset.y();
    if ((this->leg_name == "br")||(this->leg_name == "bl")) this->feet_pos_offset.x() = -this->feet_pos_offset.x();

}

void FeetTrajectory::setFootContactParams(float min_prop_swing_time_before_contact_detection, float dt_contact_detection_seconds){
    this->min_prop_swing_time_before_contact_detection = min_prop_swing_time_before_contact_detection;
    this->dt_contact_detection_seconds = dt_contact_detection_seconds;
}


void FeetTrajectory::setWalking(bool walking){
    if (walking == this->walking) return;

    if (walking){
        this->walking = true;
        this->stop_request_state = NON_STOP_REQUEST;
        // this->feet_state        = STANCE;
        // this->leg_phase     = 0;
        // this->feet_state_phase = 0;
        // this->set_initial_pos  = false;
        this->timer = ros::Time::now();

    }
    else if ((!walking) && (this->stop_request_state == NON_STOP_REQUEST)){
        this->stop_request_state = REQUESTED_TO_STOP;
    }
}


bool FeetTrajectory::predictFeetState(double current_main_phase,
                                      double dt,
                                      const Eigen::Vector3d &ref_base_pos,
                                      const Eigen::Vector3d &ref_base_vel,
                                      const Eigen::Vector3d &ref_base_ang,
                                      const Eigen::Vector3d &ref_base_ang_vel,
                                      const Eigen::Vector3d &current_foot_pos,
                                      Eigen::Vector3d* predicted_foot_pos){


    if (not this->walking){
        predicted_foot_pos->operator=( current_foot_pos ); //this->current_pos );
        return true;
    }


    double phase_    = current_main_phase + dt + this->phase_offset;

    bool use_current_foot_pos = true;
    if (phase_ > 1) use_current_foot_pos = false;

    double leg_phase_ = std::fmod(phase_, 1.0);

    bool leg_stance_ = 1;

    if (leg_phase_ < prop_time_swing){   // FEET IS SWING, NO PREDiCTION DURING SWING PERIOD
        leg_stance_ = 0;
        predicted_foot_pos->operator=(  current_foot_pos );
        return leg_stance_;
    }


    Eigen::Matrix4f T_body;
    Eigen::Vector3f href_rel;
    Eigen::Vector3f center_foot;

    Eigen::Vector3f ref_base_pos_f, ref_base_ang_f;
    ref_base_pos_f = ref_base_pos.cast<float>();
    ref_base_ang_f = ref_base_ang.cast<float>();


    this->kinematics->getTransform(ref_base_pos_f, ref_base_ang_f, &T_body);
    this->kinematics->getLegRefRelativePosition(this->leg_name, &href_rel);
    this->kinematics->applyTransform(href_rel + this->feet_pos_offset, T_body, &center_foot);
    center_foot(2) = 0;

    Eigen::Vector3d cross_prod;
    cross_prod = ref_base_ang_vel.cross(center_foot.cast<double>() - ref_base_pos);

    // if (this->leg_name == "fl"){
    // std::cout << cross_prod.x() << ", " <<  cross_prod.y() << ", " << cross_prod.z() << std::endl;
    // std::cout << ref_base_ang_vel.x() << ", " <<  ref_base_ang_vel.y() << ", " << ref_base_ang_vel.z() << std::endl;
    // std::cout << center_foot.x() << ", " <<  center_foot.y() << ", " << center_foot.z() << std::endl << std::endl;

    // }

    Eigen::Vector3d foot_vel  = ref_base_vel + cross_prod;

    Eigen::Vector3d foot_dist = foot_vel * period*(1 - prop_time_swing); // total distance that the feet will displace in stance
    double t = ((leg_phase_ - prop_time_swing)/(1 - prop_time_swing)) - 0.5; // parameter for time in the range [-0.5, 0.5]


    if (use_current_foot_pos){       // The foot did not lift from the ground yet
        predicted_foot_pos->operator=(  current_foot_pos - foot_dist * t);
    }


    else{
        predicted_foot_pos->operator=(  center_foot.cast<double>() - foot_dist * t);
    }


    return leg_stance_;


}


void FeetTrajectory::getFeetState(double main_phase, bool foot_contact){
    double phase_ = main_phase + this->phase_offset;
    this->leg_phase = std::fmod(phase_, 1.0);




    if (leg_phase < prop_time_swing){
        feet_state = SWING;
        feet_state_phase = leg_phase/prop_time_swing;

        // if ((leg_phase < 0.1*prop_time_swing)&&(this->stop_request_state == REQUESTED_TO_STOP)){
        // this->stop_request_state = MADE_LAST_STEP;
        // }



        // ******** THIS IS FOOT CONTACT ****************
        // if ((leg_phase > min_prop_swing_time_before_contact_detection  *  prop_time_swing) && (foot_contact)){

        if ((feet_state_phase > min_prop_swing_time_before_contact_detection) && (foot_contact)){

            if (foot_contact){
                if (!prev_contact){
                    this->contactTimer = ros::Time::now();
                }

                double dt =  (ros::Time::now() - this->contactTimer).toSec();

                if (dt > dt_contact_detection_seconds){
                    feet_state = STANCE;
                    this->integrated_feet_pos << 0,0,0;


                    // if (this->leg_name == "fr") std::cout << this->leg_name << " early stance" << std::endl;

                }

            }

            prev_contact = foot_contact;


        }


        // if ((leg_phase > 0.6*prop_time_swing)&&(feet_prev_state == STANCE)){
        // feet_state = STANCE;
        // }





    }
    else{
        feet_state = STANCE;
        if (feet_prev_state == SWING){
            this->integrated_feet_pos << 0,0,0;
        }

        feet_state_phase = (leg_phase - prop_time_swing)/(1 - prop_time_swing);
        /*
        if(this->stop_request_state == MADE_LAST_STEP){

            this->stop_request_state = NON_STOP_REQUEST;
            this->walking = false;
        }
        */

        prev_contact = false;

    }

    feet_prev_state = feet_state;
}





void FeetTrajectory::makeStep(bool foot_contact, const Eigen::Vector3f &leg_des_dir, const Eigen::Vector3f &des_base_vel, const Eigen::Vector3f &base_vel, double main_phase, float base_height, Eigen::Vector3f *feet_pos){
    // get Delta t
    double dt = (ros::Time::now() - timer).toSec();
    timer = ros::Time::now();


    Eigen::Vector3f leg_des_vel, leg_des_vel_nonz, base_vel_nonz;
    leg_des_vel = leg_des_dir;
    leg_des_vel_nonz << leg_des_vel.x(), leg_des_vel.y(), 0;
    base_vel_nonz << base_vel.x(), base_vel.y(), 0;



    if (!this->walking){

        // this->current_pos = this->feet_pos_offset; // + leg_des_dir;
        this->foot_vel = leg_des_vel;
        this->current_pos_tmp = this->current_pos +  leg_des_vel*dt;
        feet_pos->operator=(  this->current_pos_tmp  );

        return;
    }




    this->getFeetState(main_phase, foot_contact);


    if (this->feet_state == STANCE){

        this->foot_vel = leg_des_vel;
        this->integrated_feet_pos = this->integrated_feet_pos + leg_des_vel * dt;
        this->current_pos_tmp = this->current_pos + leg_des_vel * dt;
        this->prev_feet_vel = leg_des_vel_nonz;
        // this->current_pos = this->current_pos + leg_ref_vel_des_nonz*dt; // - (base_vel_nonz - (leg_ref_vel_des_nonz))*dt;
        this->set_initial_pos = false;

        float dist_to_feet_offset = (this->current_pos_tmp  - feet_pos_offset + this->base_pos_offset).norm();


        if (this->stop_request_state == REQUESTED_TO_STOP){
            if (dist_to_feet_offset < 0.01){
                this->walking = false;
            }
        }

        if (this->leg_name == "fr"){
            // std::cout << this->current_pos_tmp(0) << std::endl;
        }


    }
    else{
        if (!this->set_initial_pos){
            this->initial_pos.operator=(this->current_pos);
            // this->initial_pos(2) = 0;
            this->set_initial_pos = true;
        }

        if (this->feet_state_phase < 0.1){                      // change final position only at the beginning of the step
            this->final_pos << feet_pos_offset - this->base_pos_offset + des_base_vel * (period/2) * (1 - this->prop_time_swing) + this->capure_point_weight * ( base_vel_nonz - des_base_vel)*(pow(height/9.8, 0.5));

        }

        Eigen::Vector3f middle_pos;
        middle_pos    = this->initial_pos + (this->final_pos - this->initial_pos).operator/(2);
        middle_pos(2) = middle_pos(2) + height;
        // middle_pos(2) -= 0.02;
        double t = (this->leg_phase/prop_time_swing);
        t = t*PI;
        t = 1- 0.5*(cos(t) + 1);  // Use S curve (cos(t)) for acceleration


        // float tau = 0.25;
        // t = (1 - exp(-t/tau) )/(1-exp(-1/tau));

        float x = pow(1 - t, 2)*this->initial_pos(0) + 2*t*(1-t)*middle_pos(0) + pow(t,2)*this->final_pos(0);
        float y = pow(1 - t, 2)*this->initial_pos(1) + 2*t*(1-t)*middle_pos(1) + pow(t,2)*this->final_pos(1);
        float z = pow(1 - t, 2)*this->initial_pos(2) + 2*t*(1-t)*middle_pos(2) + pow(t,2)*this->final_pos(2);

        if (this->leg_name == "fr"){
            // std::cout << this->initial_pos(0) <<   " " << this->final_pos(0) <<  " dif = " << this->final_pos(0) - this->initial_pos(0) << std::endl;
            // std::cout << "finishes here" << std::endl;
        }

        Eigen::Vector3f cmd_pos;
        cmd_pos << x,y,z;

        this->foot_vel = (this->current_pos_tmp - cmd_pos)/dt;
        this->current_pos_tmp << x,y,z;
    }


    feet_pos->operator=(this->current_pos_tmp);




}



float FeetTrajectory::getWeightingFactor(float sigma1, float sigma2, float sigma1_, float sigma2_, float offset){

    float k_stance = offset + 3*sigma1*pow(1-this->feet_state_phase,2)* this->feet_state_phase + 3*sigma2*(1-this->feet_state_phase)*pow(this->feet_state_phase, 2);
    float k_swing  = offset - 3*sigma1_*pow(1-this->feet_state_phase,2)* this->feet_state_phase - 3*sigma2_*(1-this->feet_state_phase)*pow(this->feet_state_phase, 2);

    if (k_stance > 1) k_stance = 1;
    if (k_swing < 0) k_swing = 0;

    float factor;
    if (this->feet_state == STANCE) {factor =  k_stance;}
    else{ factor =  k_swing;}

    return factor;
}


bool FeetTrajectory::getStance(){
    if (this->feet_state == STANCE){
        return 1;
    }
    else{
        return 0;
    }
}






