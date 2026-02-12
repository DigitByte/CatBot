//
// Created by andres on 11.10.20.
//

#include "../include/kinematics.h"

Kinematics::Kinematics(std::string urdf_path, std::string legs_format){
    this->urdf_path = urdf_path;
    this->legs_format = legs_format;
}
Kinematics::~Kinematics(){

}

void Kinematics::getLegDimentions(){
    urdf::Model model;

    if (!model.initFile(this->urdf_path)){
        ROS_ERROR("Failed to parse urdf file");
        return;
    }
    urdf::JointConstSharedPtr j_ref  = model.getJoint("fl_ref");
    urdf::JointConstSharedPtr j_hip1 = model.getJoint("hip1_fl");
    urdf::JointConstSharedPtr j_hip2 = model.getJoint("hip2_fl");
    urdf::JointConstSharedPtr j_knee = model.getJoint("knee_fl");
    urdf::JointConstSharedPtr j_feet = model.getJoint("feet_fl_ref");


    this->offset_ref <<
        j_ref->parent_to_joint_origin_transform.position.x,
        j_ref->parent_to_joint_origin_transform.position.y,
        j_ref->parent_to_joint_origin_transform.position.z;

    this->offset_hip1 <<
        j_hip1->parent_to_joint_origin_transform.position.x,
        j_hip1->parent_to_joint_origin_transform.position.y,
        j_hip1->parent_to_joint_origin_transform.position.z;

    this->offset_hip2 <<
        j_hip2->parent_to_joint_origin_transform.position.x,
        j_hip2->parent_to_joint_origin_transform.position.y,
        j_hip2->parent_to_joint_origin_transform.position.z;

    this->offset_knee <<
        j_knee->parent_to_joint_origin_transform.position.x,
        j_knee->parent_to_joint_origin_transform.position.y,
        j_knee->parent_to_joint_origin_transform.position.z;

    this->offset_feet <<
        j_feet->parent_to_joint_origin_transform.position.x,
        j_feet->parent_to_joint_origin_transform.position.y,
        j_feet->parent_to_joint_origin_transform.position.z;

    this->T_body << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;

    ROS_INFO("Successfully parsed urdf file");
}

double Kinematics::constrainAngle(double x){
    x = std::fmod(x + PI, 2*PI);
    if (x < 0)
        x += 2*PI;
    return x - PI;
}


bool Kinematics::IK_leg_fromRef(std::string leg, const Eigen::Vector3f &feet_pos, Eigen::Vector3f* angles){

    // std::cout << "IK: " <<  leg << " " << feet_pos.x() << " " << feet_pos.y() << " " << feet_pos.z() << std::endl;


    float L0 = abs( this->offset_hip1.y() + this->offset_hip2.y() + this->offset_knee.y() + this->offset_feet.y() );  // Offset of the feet in the y axis
    float L1 = abs( this->offset_knee.z() );           // Length of the femur in the z axis
    float L2 = abs( this->offset_feet.z() );           // Length of the tibia in the z axis


    double z_corr_2 = pow(feet_pos.y(), 2) + pow(feet_pos.z(), 2) - pow(L0, 2);
    double D        = (pow(feet_pos.x(), 2) + pow(feet_pos.y(), 2) + pow(feet_pos.z(), 2) - pow(L0, 2) - pow(L1, 2) - pow(L2, 2))/(2*L1*L2);

    if((D > 1) || (z_corr_2 < 0)) {   // Reachability not possible
        ROS_ERROR("Reachability not possible");
        // std::cout << leg << ": " <<  feet_pos.x() << " " << feet_pos.y() << " " << feet_pos.z() <<  std::endl;
        return false;
    }

    double z_corr = pow(z_corr_2, 0.5);

    double a;
    if ((leg == "fl")||(leg == "bl")){
        a = PI - atan2(-feet_pos.z(), feet_pos.y()) - atan2(z_corr, -L0);  // hip1 angle
    }
    else{
        a = -( - atan2(feet_pos.z(), feet_pos.y()) - atan2(z_corr, -L0) );  // hip1 angle
    }


    double c = atan2(- pow(1 - pow(D, 2),0.5), D); // knee angle
    double b = -PI + atan2(feet_pos.x(), -z_corr) - atan2(L2 * sin(c), L1 + L2 * cos(c));  // hip2 angle

    if (this->legs_format == "inward"){
        if ((leg == "bl")||(leg == "br")){
            c = -atan2(- pow(1 - pow(D, 2),0.5), D); // knee angle
            b = (PI + atan2(feet_pos.x(), -z_corr) - atan2(L2 * sin(c), L1 + L2 * cos(c)) );  // hip2 angle
        }
    }



    angles->operator()(0) = this->constrainAngle(a);
    angles->operator()(1) = this->constrainAngle(b);
    angles->operator()(2) = this->constrainAngle(c);

    return true;

}


void Kinematics::FK_leg_fromRef(std::string leg, const Eigen::Vector3f &angles, Eigen::Vector3f* feet_pos){
    float L0 = abs( this->offset_hip1.y() + this->offset_hip2.y() + this->offset_knee.y() + this->offset_feet.y() );  // Offset of the feet in the y axis
    float L0_x = abs( this->offset_hip1.x() + this->offset_hip2.x() + this->offset_knee.x() + this->offset_feet.x() );
    float L1         = abs(offset_knee.z());  // Length of the femur in the z axis
    float L2         = abs(offset_feet.z());  // Length of the tibia in the z axis


    if ((leg == "fr")||(leg == "br")) L0 = -L0;
    if ((leg == "br")||(leg == "bl")) L0_x = -L0_x;

    feet_pos->operator()(0) = -L1*sin(angles(1)) - L2*sin(angles(1) + angles(2)) + L0_x;
    feet_pos->operator()(1) =  L1*sin(angles(0))*cos(angles(1)) + L2*sin(angles(0))*cos(angles(1) + angles(2)) + L0*cos(angles(0)) ;
    feet_pos->operator()(2) =  L0*sin(angles(0)) - L1*cos(angles(0))*cos(angles(1))-L2*cos(angles(0))*cos(angles(1) + angles(2));

}


void Kinematics::getRotationMatrix(const Eigen::Vector3f &rpy, Eigen::Matrix3f* R){
    Eigen::Matrix3f Rx, Ry, Rz, Rxyz;
    Rx << 1,            0,                    0,
          0,            cos(rpy(0)), -sin(rpy(0)),
          0,            sin(rpy(0)),  cos(rpy(0));



    Rz << cos(rpy(1)),      -sin(rpy(1)),   0,
               sin(rpy(1)),  cos(rpy(1)),   0,
               0,            0,             1;



    Ry << cos(rpy(2)),             0,              sin(rpy(2)),
              0,                   1,              0,
              -sin(rpy(2)),        0,              cos(rpy(2));


    Rxyz = Rx*Ry*Rz;

    R->operator=(Rxyz);

}


void Kinematics::getTransform(const Eigen::Vector3f &pos, const Eigen::Vector3f &rpy, Eigen::Matrix4f* T){
    // std::cout << rpy(0) << " " << rpy(1) << " " << rpy(2) << std::endl;
    Eigen::Matrix4f Rx, Ry, Rz, T_pos, Rxyz;
    Rx << 1,            0,                    0,                     0,
          0,            cos(rpy(0)), -sin(rpy(0)),  0,
          0,            sin(rpy(0)),  cos(rpy(0)),  0,
          0,            0,                     0,                    1;



    Rz << cos(rpy(2)), -sin(rpy(2)),   0,             0,
          sin(rpy(2)),  cos(rpy(2)),   0,             0,
          0,            0,             1,             0,
          0,            0,             0,             1;



    Ry << cos(rpy(1)),             0,              sin(rpy(1)),   0,
          0,                   1,              0,             0,
         -sin(rpy(1)),        0,              cos(rpy(1)),   0,
              0,                   0,              0,             1;

    T_pos << 1, 0, 0, pos.x(),
             0, 1, 0, pos.y(),
             0, 0, 1, pos.z(),
             0, 0, 0, 1;

    Rxyz = Rz*Ry*Rx; //Rx*Ry*Rz;

    T->operator=(T_pos * Rxyz);
}


void Kinematics::getTransform(const Eigen::Vector3f &pos, const Eigen::Matrix3f &rot_matrix, Eigen::Matrix4f* T){
    Eigen::Matrix4f T_pos, Rxyz;

    Rxyz << rot_matrix(0,0), rot_matrix(0,1), rot_matrix(0,2), 0,
            rot_matrix(1,0), rot_matrix(1,1), rot_matrix(1,2), 0,
            rot_matrix(2,0), rot_matrix(2,1), rot_matrix(2,2), 0,
            0,               0,               0,               1;


    T_pos << 1, 0, 0, pos.x(),
             0, 1, 0, pos.y(),
             0, 0, 1, pos.z(),
             0, 0, 0, 1;

    T->operator=(T_pos * Rxyz);

}



void Kinematics::applyTransform(const Eigen::Vector3f &v, const Eigen::Matrix4f &transform, Eigen::Vector3f *v_trans){
    Eigen::Vector4f v4d, v4d_trans;
    v4d << v.x(), v.y(), v.z(), 1;
    v4d_trans = transform * v4d;
    Eigen::Vector3f v_trans_;
    v_trans_ << v4d_trans(0), v4d_trans(1), v4d_trans(2);
    v_trans->operator=(v_trans_);
}

void Kinematics::getLegRefRelativePosition(std::string leg, Eigen::Vector3f* href){
    Eigen::Vector3f href_;
    href_(0) = this->offset_ref.x();
    href_(1) = this->offset_ref.y();
    href_(2) = this->offset_ref.z();

    if ((leg == "br")||(leg == "bl"))  href_(0) = -href_(0);
    if ((leg == "fr")||(leg == "br"))  href_(1) = -href_(1);

    href->operator=( href_ );
}

void Kinematics::getLegRefPosition(std::string leg, Eigen::Vector3f* href){
    Eigen::Vector4f href_;
    href_(0) = this->offset_ref.x();
    href_(1) = this->offset_ref.y();
    href_(2) = this->offset_ref.z();
    href_(3) = 1;

    if ((leg == "br")||(leg == "bl"))  href_(0) = -href_(0);
    if ((leg == "fr")||(leg == "br"))  href_(1) = -href_(1);


    href_ = this->T_body*href_;
    href->operator()(0) = href_(0);
    href->operator()(1) = href_(1);
    href->operator()(2) = href_(2);
}

void Kinematics::setBodyPose( const Eigen::Vector3f &body_pos, const Eigen::Vector3f &body_rpy){
    this->getTransform(body_pos, body_rpy, &this->T_body);
}





bool Kinematics::feetAnglesFromRef(std::string leg, const Eigen::Vector3f &feet_pos_from_ref, Eigen::Vector3f* angles){
    Eigen::Vector3f href, href_from_body, feet_pos;

    href(0) = this->offset_ref.x();
    href(1) = this->offset_ref.y();
    href(2) = this->offset_ref.z();
    if ((leg == "br")||(leg == "bl"))  href(0) = -href(0);
    if ((leg == "fr")||(leg == "br"))  href(1) = -href(1);

    this->getLegRefPosition(leg, &href_from_body);

    // std::cout << leg << " pos from ref: " << feet_pos_from_ref.x() << feet_pos_from_ref.y() << feet_pos_from_ref.z() << std::endl;

    feet_pos(0) = feet_pos_from_ref(0) + href(0)- href_from_body(0);
    feet_pos(1) = feet_pos_from_ref(1) + href(1)- href_from_body(1);
    feet_pos(2) = feet_pos_from_ref(2) + href(2)- href_from_body(2);



    // std::cout << leg << " pos feet: " << feet_pos.x() << feet_pos.y() << feet_pos.z() << std::endl;

    bool hr = this->IK_leg_fromRef(leg, feet_pos, angles);
    return hr;
}




void Kinematics::feetPosFromRef(std::string leg, const Eigen::Vector3f &angles, Eigen::Vector3f* feet_pos_from_ref){
    Eigen::Vector3f href, href_from_body, feet_pos;

    this->FK_leg_fromRef(leg, angles, &feet_pos);

    feet_pos_from_ref->operator=(  feet_pos );

    /*
    href(0) = this->offset_ref.x();
    href(1) = this->offset_ref.y();
    href(2) = this->offset_ref.z();
    if ((leg == "br")||(leg == "bl"))  href(0) = -href(0);
    if ((leg == "fr")||(leg == "br"))  href(1) = -href(1);

    this->getLegRefPosition(leg, &href_from_body);

    feet_pos_from_ref->operator=(  feet_pos -  href + href_from_body );
    */
}

/**
 * This funcion gets the feet position respect to the base (not respect to the projection of the base to the ground)
 * @param leg
 * @param angles
 * @param feet_pos_from_ref
 */
void Kinematics::feetPosFromBase(std::string leg, const Eigen::Vector3f &angles, Eigen::Vector3f* feet_pos_from_base){
    Eigen::Vector3f href, href_from_body, feet_pos;

    this->FK_leg_fromRef(leg, angles, &feet_pos);

    // std::cout << "FK: " <<  leg << " " << feet_pos.x() << " " << feet_pos.y() << " " << feet_pos.z() << std::endl;

    href(0) = this->offset_ref.x();
    href(1) = this->offset_ref.y();
    href(2) = this->offset_ref.z();
    if ((leg == "br")||(leg == "bl"))  href(0) = -href(0);
    if ((leg == "fr")||(leg == "br"))  href(1) = -href(1);

    feet_pos_from_base->operator=(  feet_pos +  href );
}






void Kinematics::getFeetFromBody(std::string leg, const Eigen::Vector3f &feet_pos_from_ref, Eigen::Vector3f* feet_pos_from_body){
    Eigen::Vector3f href;
    href(0) = this->offset_ref.x();
    href(1) = this->offset_ref.y();
    href(2) = this->offset_ref.z();
    if ((leg == "br")||(leg == "bl"))  href(0) = -href(0);
    if ((leg == "fr")||(leg == "br"))  href(1) = -href(1);


    feet_pos_from_body->operator=( feet_pos_from_ref + href );
}

void Kinematics::getRefVelFromBodyVel(std::string leg, const Eigen::Vector3f &body_vel, Eigen::Vector3f* ref_vel){
    Eigen::Vector3f href;
    href(0) = this->offset_ref.x();
    href(1) = this->offset_ref.y();
    href(2) = this->offset_ref.z();
    if ((leg == "br")||(leg == "bl"))  href(0) = -href(0);
    if ((leg == "fr")||(leg == "br"))  href(1) = -href(1);


    ref_vel->operator()(0) = body_vel(0) ;
    ref_vel->operator()(1) = body_vel(1);

}



void Kinematics::getJacobian(std::string leg, const Eigen::Vector3f &joint_angles, Eigen::Matrix3f* Jacobian){
    float y_offset = abs(offset_hip2.y() + offset_knee.y() + offset_feet.y());
    float L1       = abs(offset_knee.z());
    float L2       = abs(offset_feet.z());
    if ((leg == "fr")||(leg == "br"))  y_offset = -y_offset;

    Jacobian->operator()(0,0) = 0;
    Jacobian->operator()(0,1) = -L1* cos(joint_angles(1)) - L2* cos(joint_angles(1) + joint_angles(2));
    Jacobian->operator()(0,2) = -L2* cos(joint_angles(1) + joint_angles(2));


    Jacobian->operator()(1,0) = - y_offset * sin(  joint_angles(0) ) + L1*cos(joint_angles(0))*cos(joint_angles(1)) + L2*cos(joint_angles(0))*cos(joint_angles(1) + joint_angles(2));
    Jacobian->operator()(1,1) = - (  L1*sin(joint_angles(1)) + L2*sin(joint_angles(1) + joint_angles(2))  )*sin(joint_angles(0));
    Jacobian->operator()(1,2) = - L2*sin(joint_angles(0))*sin(joint_angles(1) + joint_angles(2));

    Jacobian->operator()(2,0) = L1*sin(joint_angles(0))*cos(joint_angles(1)) + L2*sin(joint_angles(0))*cos(joint_angles(1) + joint_angles(2)) + y_offset*cos(joint_angles(0));
    Jacobian->operator()(2,1) = (  L1*sin(joint_angles(1)) + L2*sin(joint_angles(1) + joint_angles(2))  )*cos(joint_angles(0));
    Jacobian->operator()(2,2) = L2*sin(joint_angles(1) + joint_angles(2))*cos(joint_angles(0));

    /*
    Jacobian->operator()(0,0) = 0;
    Jacobian->operator()(0,1) = -L1* cos(joint_angles(1)) - L2* cos(joint_angles(1) + joint_angles(2));
    Jacobian->operator()(0,2) = -L2* cos(joint_angles(1) + joint_angles(2));


    Jacobian->operator()(1,0) = 0;
    Jacobian->operator()(1,1) = 0;
    Jacobian->operator()(1,2) = 0;

    Jacobian->operator()(2,0) = 0;
    Jacobian->operator()(2,1) = L1*sin(joint_angles(1)) + L2*sin(joint_angles(1) + joint_angles(2));
    Jacobian->operator()(2,2) = L2*sin(joint_angles(1) + joint_angles(2));
    */

}

void Kinematics::getPseudoInvJacobian(std::string leg, const Eigen::Vector3f &joint_angles, Eigen::Matrix3f* InvJacobian){
    Eigen::Matrix3f Jac;
    this->getJacobian(leg, joint_angles, &Jac);

    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3f> cqr(Jac);
    InvJacobian->operator=( cqr.pseudoInverse() );
}



void Kinematics::getForceReaction(std::string leg, const Eigen::Vector3f &joint_angles, const Eigen::Vector3f &joint_torques, Eigen::Vector3f* force){
    Eigen::Matrix3f Jac, InvJacobian_T;
    this->getJacobian(leg, joint_angles, &Jac);

    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3f> cqr(Jac.transpose());
    InvJacobian_T.operator=( cqr.pseudoInverse() );

    force->operator=( InvJacobian_T * joint_torques  );

}

void Kinematics::getTorques(std::string leg, const Eigen::Vector3f &joint_angles, const Eigen::Vector3f &force, Eigen::Vector3f* joint_torques) {
    Eigen::Matrix3f Jac,Jac_T;
    this->getJacobian(leg, joint_angles, &Jac);
    Jac_T.operator=( Jac.transpose() );

    joint_torques->operator=( Jac_T  * force  );
}