#include "../include/gait_control_node.h"

void eigen2Point(geometry_msgs::Point *p, Eigen::Vector3f* vect){
    p->x = vect->operator()(0);
    p->y = vect->operator()(1);
    p->z = vect->operator()(2);

}



float clipValue(float val, float min, float max){
    if (val > max) val = max;
    if (val < min) val = min;
    return val;
}



GaitControl::GaitControl(ros::NodeHandle* n, std::string robot_name, std::string urdf_path, std::string legs_format, std::string gaits_dir, std::string sequences_dir):
    robot_state(),
    joint_state(),
    feet_press(),
    vel_cmd(),
    body_pose_cmd(),
    joint_control_msg(),
    fl_trajectory("fl"),
    fr_trajectory("fr"),
    bl_trajectory("bl"),
    br_trajectory("br"),
    filter_vel_cmd(),
    filter_vel_est()
{
	nh = n;
    this->performing_sequence = false;
	this->debug("INFO", "gait_control_node started.");
	this->robot_name    = robot_name;
    this->gaits_dir     = gaits_dir;
    this->sequences_dir = sequences_dir;
	// ros::Duration(7).sleep();

	this->robot_state.pose.position.x = 0;
    this->robot_state.pose.position.y = 0;
    this->robot_state.pose.position.z = 0;
    this->robot_state.pose.orientation.x = 0;
    this->robot_state.pose.orientation.y = 0;
    this->robot_state.pose.orientation.z = 0;
    this->robot_state.pose.orientation.w = 1;

    // std::string urdf_path = ros::package::getPath("catbot_description") + std::string("/urdf/catbot.urdf");

	this->kinematics           = new Kinematics(urdf_path, legs_format);
	this->kinematics->getLegDimentions();

	this->polygonSupport       = new PredPolygonSupport();
	this->contactDetector      = new ContactDetection();
	this->torqController       = new torqueController(this->kinematics);
	this->stateEstimation      = new StateEstimation(this->kinematics);
	this->predModelController  = new PredictiveModeController(&this->fl_trajectory, &this->fr_trajectory, &this->bl_trajectory, &this->br_trajectory, this->kinematics);
    this->predModelControllerNormal = new PredictiveModeController(&this->fl_trajectory, &this->fr_trajectory, &this->bl_trajectory, &this->br_trajectory, this->kinematics);

    this->fl_trajectory.setKinematics(this->kinematics);
    this->fr_trajectory.setKinematics(this->kinematics);
    this->bl_trajectory.setKinematics(this->kinematics);
    this->br_trajectory.setKinematics(this->kinematics);

    this->base_quaternion << 0,0,0,1;

    /*********** INITIAL STATE *****************/
    feet_fl_pos_cmd << 0,0, -BODY_INITIAL_POS_Z;
    feet_fr_pos_cmd << 0,0, -BODY_INITIAL_POS_Z;
    feet_bl_pos_cmd << 0,0, -BODY_INITIAL_POS_Z;
    feet_br_pos_cmd << 0,0, -BODY_INITIAL_POS_Z;

    bool hr_fl = this->kinematics->IK_leg_fromRef("fl", feet_fl_pos_cmd, &angles_cmd_fl);
    bool hr_fr = this->kinematics->IK_leg_fromRef("fr", feet_fr_pos_cmd, &angles_cmd_fr);
    bool hr_bl = this->kinematics->IK_leg_fromRef("bl", feet_bl_pos_cmd, &angles_cmd_bl);
    bool hr_br = this->kinematics->IK_leg_fromRef("br", feet_br_pos_cmd, &angles_cmd_br);


    float jointPos[12] = {angles_cmd_fl.x(), angles_cmd_fl.y(), angles_cmd_fl.z(),
                          angles_cmd_fr.x(), angles_cmd_fr.y(), angles_cmd_fr.z(),
                          angles_cmd_bl.x(), angles_cmd_bl.y(), angles_cmd_bl.z(),
                          angles_cmd_br.x(), angles_cmd_br.y(), angles_cmd_br.z()};

    this->imu_data.orientation.w = 1;
    this->imu_data.orientation.x = 0;
    this->imu_data.orientation.y = 0;
    this->imu_data.orientation.z = 0;
    this->imu_data.header.frame_id = "base_link";


    estimated_base_pos << 0,0, BODY_INITIAL_POS_Z;
    locked_base_pos = estimated_base_pos;

    this->stateEstimation->setInitialState(this->base_quaternion, estimated_base_pos, jointPos);
    /*******************************************/



    Eigen::Vector3f body_pos, body_rpy;
    body_pos << BODY_OFFSET_X, BODY_OFFSET_Y,BODY_INITIAL_POS_Z;
    body_rpy << BODY_INITIAL_ROLL, BODY_INITIAL_PITCH, BODY_INITIAL_YAW;
    this->joint_control_msg.name = {"hip1_fl", "hip2_fl", "knee_fl",
                                    "hip1_fr", "hip2_fr", "knee_fr",
                                    "hip1_bl", "hip2_bl", "knee_bl",
                                    "hip1_br", "hip2_br", "knee_br"};

    this->joint_control_msg.position = {angles_cmd_fl.x(), angles_cmd_fl.y(), angles_cmd_fl.z(),
                                        angles_cmd_fr.x(), angles_cmd_fr.y(), angles_cmd_fr.z(),
                                        angles_cmd_bl.x(), angles_cmd_bl.y(), angles_cmd_bl.z(),
                                        angles_cmd_br.x(), angles_cmd_br.y(), angles_cmd_br.z()};
    this->joint_control_msg.velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
    this->joint_control_msg.effort   = {0,0,0,0,0,0,0,0,0,0,0,0};


    this->joint_state.name = {"hip1_fl", "hip2_fl", "knee_fl",
                              "hip1_fr", "hip2_fr", "knee_fr",
                              "hip1_bl", "hip2_bl", "knee_bl",
                              "hip1_br", "hip2_br", "knee_br"};

    this->joint_state.position = this->joint_control_msg.position;
    this->joint_state.velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
    this->joint_state.effort   = {0,0,0,0,0,0,0,0,0,0,0,0};


    this->getLegStateAngles("fl", &this->prev_angles_fl);
    this->getLegStateAngles("fr", &this->prev_angles_fr);
    this->getLegStateAngles("bl", &this->prev_angles_bl);
    this->getLegStateAngles("br", &this->prev_angles_br);
    set_first_ang_vel = false;

    vel_angles_fl.setZero();
    vel_angles_fr.setZero();
    vel_angles_bl.setZero();
    vel_angles_br.setZero();

    this->feet_press.leg_name = {"fl", "fr", "bl", "br"};
    geometry_msgs::Point p;
    this->feet_press.pressure_vector = {p, p, p, p};
    this->feet_press.feet_pos = {p, p, p, p};
    this->feet_press.feet_stance = {1,1,1,1};

    this->feet_forces_msg.leg_name = {"fl", "fr", "bl", "br"};
    this->feet_forces_msg.pressure_vector = {p, p, p, p};
    this->feet_forces_msg.feet_pos = {p, p, p, p};
    this->feet_forces_msg.feet_stance = {1,1,1,1};

    // [publishers]
    this->jointControlPub     = nh->advertise<sensor_msgs::JointState>( "/" + this->robot_name + "/joints_control", 10 );
    this->feetPressurePub     = nh->advertise<catbot_msgs::feet_pressure>( "/" + this->robot_name + "/feet_pressure",   10 );
    this->feetForceControlPub = nh->advertise<catbot_msgs::feet_pressure>( "/" + this->robot_name + "/feet_pressure_control",   10 );
    this->odometryPub         = nh->advertise<nav_msgs::Odometry>("/" + this->robot_name + "/odometry", 10);
	// [subscribers]
	this->robotStateSub   = nh->subscribe("/" + this->robot_name + "/state_estimation", 1, &GaitControl::robotStateCB, this);
	this->imuDataSub      = nh->subscribe("/" + this->robot_name + "/imu_data",         1, &GaitControl::imuDataCB,    this);

    this->torsoControlSub     = nh->subscribe("/" + this->robot_name + "/torso_pose_control", 1, &GaitControl::torsoControlCB, this);
    this->velCmdSub           = nh->subscribe("/" + this->robot_name + "/vel_cmd",      1, &GaitControl::velCmdCB,     this);
    this->jointStateSub       = nh->subscribe("/" + this->robot_name + "/joints_state", 1, &GaitControl::jointStateCB, this);
    this->contactDetectionSub = nh->subscribe("/" + this->robot_name + "/contact_detection", 1, &GaitControl::contactDetectionCB, this);
    // [services]

    this->setGaitService     = nh->advertiseService("/" + this->robot_name + "/set_gait",     &GaitControl::setGaitCB,     this);
    this->setSequenceService = nh->advertiseService("/" + this->robot_name + "/set_sequence", &GaitControl::setSequenceCB, this);


    // [clients]
    this->connectMotorsClient = nh->serviceClient<catbot_msgs::connect_servos>("/" + this->robot_name + "/connect_servos");

    this->walk = false;
    this->set_initial_gait_file = false;
    this->setGaitFromFile("stop");
    this->set_initial_gait_file = true;

    this->imu_data.orientation.x = 0;
    this->imu_data.orientation.y = 0;
    this->imu_data.orientation.z = 0;
    this->imu_data.orientation.w = 1;


    this->initPhase();
    this->publishControlTimer = ros::Time::now();
    this->publishVisTimer     = ros::Time::now();

    this->dtTimer = ros::Time::now();

    this->available_imu_data = false;
    this->dtTimer_IMU = ros::Time::now();

    this->loop_counter = 0;
    this->robot_COM_z_pos = BODY_INITIAL_POS_Z;


    this->contact_fl = 0;
    this->contact_fr = 0;
    this->contact_bl = 0;
    this->contact_br = 0;

    this->des_base_vel.setZero();



    this->des_base_rot << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1;


}

GaitControl::~GaitControl(){
	delete nh;
	nh = NULL;
}



/************* callbacks *******************/

void GaitControl::robotStateCB(const catbot_msgs::robot_state::ConstPtr &msg){
    this->robot_state = *msg;
}

void GaitControl::imuDataCB(const sensor_msgs::Imu::ConstPtr &msg){
    this->imu_data = *msg;
    this->available_imu_data = true;

}

void GaitControl::contactDetectionCB(const catbot_msgs::contact_detection::ConstPtr &msg){
    this->contact_detection_msg = *msg;
    this->contact_fl = this->contact_detection_msg.feet_stance[0];
    this->contact_fr = this->contact_detection_msg.feet_stance[1];
    this->contact_bl = this->contact_detection_msg.feet_stance[2];
    this->contact_br = this->contact_detection_msg.feet_stance[3];

    // std::cout << contact_fl << ", "
    // << contact_fr << ", "
    // << contact_bl << ", "
    // << contact_br << std::endl;
}

void GaitControl::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg){
    this->joint_state = *msg;
}
void GaitControl::velCmdCB(const geometry_msgs::Twist::ConstPtr &msg){

    geometry_msgs::TwistStamped cmd_vel;

    cmd_vel.twist.linear.x = clipValue( msg->linear.x, -this->max_abs_vel_x, this->max_abs_vel_x  );
    cmd_vel.twist.linear.y = clipValue( msg->linear.y, -this->max_abs_vel_y, this->max_abs_vel_y  );
    cmd_vel.twist.linear.z = clipValue( msg->linear.z, -this->max_abs_vel_z, this->max_abs_vel_z  );

    cmd_vel.twist.angular.x = clipValue( msg->angular.x, -this->max_abs_vel_roll, this->max_abs_vel_roll  );
    cmd_vel.twist.angular.y = clipValue( msg->angular.y, -this->max_abs_vel_pitch, this->max_abs_vel_pitch  );
    cmd_vel.twist.angular.z = clipValue( msg->angular.z, -this->max_abs_vel_yaw, this->max_abs_vel_yaw  );


    // ERROR HERE!
    filter_vel_cmd.update(&cmd_vel);
    // filter_vel_cmd.get(&this-> vel_cmd);
    this-> vel_cmd = filter_vel_cmd.get();

    // vel_cmd = cmd_vel;
}
void GaitControl::torsoControlCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    this->body_pose_cmd = *msg;
    tf::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    Eigen::Vector3f body_pos, body_rpy;
    body_rpy << roll, pitch, yaw;
    body_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    // this->kinematics->setBodyPose(body_pos, body_rpy);
}


bool GaitControl::setGaitCB(catbot_msgs::set_gait::Request &req, catbot_msgs::set_gait::Response &res){
    this->setGaitFromFile(req.gait_filename);

    res.result = true;
    res.info   = "setting gait Successfully";
    return true;
}


bool GaitControl::setSequenceCB(catbot_msgs::set_gait::Request &req, catbot_msgs::set_gait::Response &res){
    this->performing_sequence = true;
    this->setSequenceFromFile(req.gait_filename);
    this->performing_sequence = false;
    res.result = true;
    res.info   = "set sequence Successfully";
    return true;
}



/*
void GaitControl::subCallback(const std_msgs::String::ConstPtr &msg){
	this->debug("SUBCALLBACK", msg->data);
}

*/

void GaitControl::initPhase(){

    this->phase = 0;
    this->phaseTimer = ros::Time::now();
    this->phase_counter = 0;
}

void GaitControl::getPhase(double period){
    // double dt =  (ros::Time::now() - this->phaseTimer).toSec();
    // this->phase_counter = (int) dt/period;
    // double t = dt / period - this->phase_counter;
    // this->phase = std::fmod(dt, period)/period;
    // std::cout << this->phase << std::endl;


    double dt =  (ros::Time::now() - this->phaseTimer).toSec();
    if (dt > period){
        dt = dt - period;
        this->phaseTimer = ros::Time::now();
    }

    this->phase = dt / period;



}


void GaitControl::getLegStateAngles (std::string leg, Eigen::Vector3f* angles){
    if (leg == "fl"){
        angles->operator()(0) = this->joint_control_msg.position[0];
        angles->operator()(1) = this->joint_control_msg.position[1];
        angles->operator()(2) = this->joint_control_msg.position[2];
    }
    if (leg == "fr"){
        angles->operator()(0) = this->joint_control_msg.position[3];
        angles->operator()(1) = this->joint_control_msg.position[4];
        angles->operator()(2) = this->joint_control_msg.position[5];
    }
    if (leg == "bl"){
        angles->operator()(0) = this->joint_control_msg.position[6];
        angles->operator()(1) = this->joint_control_msg.position[7];
        angles->operator()(2) = this->joint_control_msg.position[8];
    }
    if (leg == "br"){
        angles->operator()(0) = this->joint_control_msg.position[9];
        angles->operator()(1) = this->joint_control_msg.position[10];
        angles->operator()(2) = this->joint_control_msg.position[11];
    }


}
void GaitControl::getLegStateTorques(std::string leg, Eigen::Vector3f* torques){
    if (leg == "fl"){
        torques->operator()(0) = this->joint_state.effort[0];
        torques->operator()(1) = this->joint_state.effort[1];
        torques->operator()(2) = -this->joint_state.effort[2];
    }
    if (leg == "fr"){
        torques->operator()(0) = this->joint_state.effort[3];
        torques->operator()(1) = this->joint_state.effort[4];
        torques->operator()(2) = -this->joint_state.effort[5];
    }
    if (leg == "bl"){
        torques->operator()(0) = this->joint_state.effort[6];
        torques->operator()(1) = this->joint_state.effort[7];
        torques->operator()(2) = -this->joint_state.effort[8];
    }
    if (leg == "br"){
        torques->operator()(0) = this->joint_state.effort[9];
        torques->operator()(1) = this->joint_state.effort[10];
        torques->operator()(2) = -this->joint_state.effort[11];
    }
}






void GaitControl::computeRobotState(float dt){
    /*********************************************************************************
     ************************** GET JOINT FEEDBACK ***********************************
     *********************************************************************************/

    /************ GET CURRENT ANGLES **************/
    prev_angles_fl = curr_angles_fl;
    prev_angles_fr = curr_angles_fr;
    prev_angles_bl = curr_angles_bl;
    prev_angles_br = curr_angles_br;


    this->getLegStateAngles("fl", &curr_angles_fl);
    this->getLegStateAngles("fr", &curr_angles_fr);
    this->getLegStateAngles("bl", &curr_angles_bl);
    this->getLegStateAngles("br", &curr_angles_br);

    Eigen::Vector3f prev_feet_fl_pos, prev_feet_fr_pos, prev_feet_bl_pos, prev_feet_br_pos;
    /************ GET PREVIOUS FEET POS (FROM REF) **************/
    this->kinematics->feetPosFromRef("fl", prev_angles_fl, &prev_feet_fl_pos);
    this->kinematics->feetPosFromRef("fr", prev_angles_fr, &prev_feet_fr_pos);
    this->kinematics->feetPosFromRef("bl", prev_angles_bl, &prev_feet_bl_pos);
    this->kinematics->feetPosFromRef("br", prev_angles_br, &prev_feet_br_pos);

    /************ GET CURRENT FEET POS (FROM REF) **************/
    this->kinematics->feetPosFromRef("fl", curr_angles_fl, &curr_feet_fl_pos);
    this->kinematics->feetPosFromRef("fr", curr_angles_fr, &curr_feet_fr_pos);
    this->kinematics->feetPosFromRef("bl", curr_angles_bl, &curr_feet_bl_pos);
    this->kinematics->feetPosFromRef("br", curr_angles_br, &curr_feet_br_pos);


    /************ GET CURRENT FEET PO (FROM BASE) **************/
    this->kinematics->feetPosFromBase("fl", curr_angles_fl, &curr_feet_fl_pos_fromBase);
    this->kinematics->feetPosFromBase("fr", curr_angles_fr, &curr_feet_fr_pos_fromBase);
    this->kinematics->feetPosFromBase("bl", curr_angles_bl, &curr_feet_bl_pos_fromBase);
    this->kinematics->feetPosFromBase("br", curr_angles_br, &curr_feet_br_pos_fromBase);

    /************ GET CURRENT TORQUES **************/
    Eigen::Vector3f curr_torques_fl, curr_torques_fr, curr_torques_bl, curr_torques_br;
    this->getLegStateTorques("fl", &curr_torques_fl);
    this->getLegStateTorques("fr", &curr_torques_fr);
    this->getLegStateTorques("bl", &curr_torques_bl);
    this->getLegStateTorques("br", &curr_torques_br);

    /************ GET CURRENT FEET FORCE **************/
    Eigen::Vector3f curr_force_fl, curr_force_fr, curr_force_bl, curr_force_br;
    this->kinematics->getForceReaction("fl", curr_angles_fl, curr_torques_fl, &curr_force_fl);
    this->kinematics->getForceReaction("fr", curr_angles_fr, curr_torques_fr, &curr_force_fr);
    this->kinematics->getForceReaction("bl", curr_angles_bl, curr_torques_bl, &curr_force_bl);
    this->kinematics->getForceReaction("br", curr_angles_br, curr_torques_br, &curr_force_br);


    /************ GET CURRENT CONSTANT STANCE **************/
    curr_stance_fl = this->fl_trajectory.getStance();//this->contactDetector->isFeetStanding(&curr_force_fl);
    curr_stance_fr = this->fr_trajectory.getStance();//this->contactDetector->isFeetStanding(&curr_force_fr);
    curr_stance_bl = this->bl_trajectory.getStance();//this->contactDetector->isFeetStanding(&curr_force_bl);
    curr_stance_br = this->br_trajectory.getStance();//this->contactDetector->isFeetStanding(&curr_force_br);


    /**************************************************************************
    ***************************** STATE ESTIMATION ****************************
    ****************************************************************************/

    float dt_imu = (ros::Time::now() - this->dtTimer_IMU).toSec();
    this->dtTimer_IMU = ros::Time::now();

    base_quaternion << this->imu_data.orientation.x,
                       this->imu_data.orientation.y,
                       this->imu_data.orientation.z,
                       this->imu_data.orientation.w;


    Eigen::Vector3f acc;
    acc << this->imu_data.linear_acceleration.x,
           this->imu_data.linear_acceleration.y,
           this->imu_data.linear_acceleration.z;


    bool feet_stance[4] = {curr_stance_fl, curr_stance_fr, curr_stance_bl, curr_stance_br};

    this->stateEstimation->setInput(base_quaternion, acc, feet_stance, dt_imu);
    this->stateEstimation->predictState();

    float joint_pos[12] = {curr_angles_fl.x(), curr_angles_fl.y(), curr_angles_fl.z(),
                           curr_angles_fr.x(), curr_angles_fr.y(), curr_angles_fr.z(),
                           curr_angles_bl.x(), curr_angles_bl.y(), curr_angles_bl.z(),
                           curr_angles_br.x(), curr_angles_br.y(), curr_angles_br.z()};

    float joint_vel[12] = {vel_angles_fl.x(), vel_angles_fl.y(), vel_angles_fl.z(),
                           vel_angles_fr.x(), vel_angles_fr.y(), vel_angles_fr.z(),
                           vel_angles_bl.x(), vel_angles_bl.y(), vel_angles_bl.z(),
                           vel_angles_br.x(), vel_angles_br.y(), vel_angles_br.z()};


    float feet_height[4] = {0, 0, 0, 0};

    Eigen::Vector3f ang_vel;
    ang_vel << this->imu_data.angular_velocity.x,
               this->imu_data.angular_velocity.y,
               this->imu_data.angular_velocity.z;



    // this->stateEstimation->setFeetVel("fl", des_base_vel);// -this->normalize_stance_time * des_feet_dir_fl);
    // this->stateEstimation->setFeetVel("fr", des_base_vel);//-this->normalize_stance_time * des_feet_dir_fr);
    // this->stateEstimation->setFeetVel("bl", des_base_vel);//-this->normalize_stance_time * des_feet_dir_bl);
    // this->stateEstimation->setFeetVel("br", des_base_vel);//-this->normalize_stance_time * des_feet_dir_br);

    // Get feet velocity

    Eigen::Vector3f curr_vel_fl, curr_vel_fr, curr_vel_bl, curr_vel_br;
    // this->fl_trajectory.getVelocityFromBase(&curr_vel_fl);
    // this->fr_trajectory.getVelocityFromBase(&curr_vel_fr);
    // this->bl_trajectory.getVelocityFromBase(&curr_vel_bl);
    // this->br_trajectory.getVelocityFromBase(&curr_vel_br);


    curr_vel_fl = (curr_feet_fl_pos - prev_feet_fl_pos)/dt;
    curr_vel_fr = (curr_feet_fr_pos - prev_feet_fr_pos)/dt;
    curr_vel_bl = (curr_feet_bl_pos - prev_feet_bl_pos)/dt;
    curr_vel_br = (curr_feet_br_pos - prev_feet_br_pos)/dt;


    // std::cout << "fl" << ": " <<  curr_vel_fl.x() << " " << curr_vel_fl.y() << " " << curr_vel_fl.z() <<  std::endl;
    // std::cout << "fr" << ": " <<  curr_vel_fr.x() << " " << curr_vel_fr.y() << " " << curr_vel_fr.z() <<  std::endl;
    // std::cout << "bl" << ": " <<  curr_vel_bl.x() << " " << curr_vel_bl.y() << " " << curr_vel_bl.z() <<  std::endl;
    // std::cout << "br" << ": " <<  curr_vel_br.x() << " " << curr_vel_br.y() << " " << curr_vel_br.z() <<  std::endl;



    if (!set_first_ang_vel){
        curr_vel_fl.setZero();
        curr_vel_fr.setZero();
        curr_vel_bl.setZero();
        curr_vel_br.setZero();
        set_first_ang_vel = true;

    }
    // curr_vel_fl.setZero();
    // curr_vel_fr.setZero();
    // curr_vel_bl.setZero();
    // curr_vel_br.setZero();
    // ros::Duration(0.1).sleep();






    this->stateEstimation->setFeetVel("fl", this->base_rotation_matrix * (curr_vel_fl));
    this->stateEstimation->setFeetVel("fr", this->base_rotation_matrix * (curr_vel_fr));
    this->stateEstimation->setFeetVel("bl", this->base_rotation_matrix * (curr_vel_bl));
    this->stateEstimation->setFeetVel("br", this->base_rotation_matrix * (curr_vel_br));


    this->stateEstimation->setMeasurement(ang_vel, joint_pos, joint_vel, feet_height);

    this->stateEstimation->updateState();

    Eigen::Vector3f estimated_base_pos_prev;
    estimated_base_pos_prev << estimated_base_pos.x(), estimated_base_pos.y(), estimated_base_pos.z();

    this->stateEstimation->getBasePosition(&estimated_base_pos);
    this->stateEstimation->getBaseVelocity(&estimated_base_vel);


    this->stateEstimation->getBaseAccelerationFromBaseRef(&base_acc_from_base);
    this->stateEstimation->getBaseRotationMatrix(&base_rotation_matrix);


    // std::cout << "actual velocity x " << (estimated_base_pos.x() - estimated_base_pos_prev.x())/dt << std::endl;

    tf::Matrix3x3 base_rotation_matrix_tf;
    tf::matrixEigenToTF(base_rotation_matrix.cast <double> (), base_rotation_matrix_tf);
    double yaw, pitch, roll;
    base_rotation_matrix_tf.getEulerYPR(yaw, pitch, roll);
    base_euler_angles << roll, pitch, yaw;





    this->stateEstimation->getBaseAngularVelocity(&estimated_base_ang_vel);

    // float delay_time = -0.1;
    // estimated_base_pos = estimated_base_pos + estimated_base_vel*delay_time;
    // base_euler_angles  = base_euler_angles  + estimated_base_ang_vel*delay_time;

    // std::cout << "Estimated ang vel: " << estimated_base_ang_vel.x() << " " << estimated_base_ang_vel.y() << " " << estimated_base_ang_vel.z() << std::endl;
    // std::cout << "Estimated ang: " << roll << " " << pitch << " " << yaw << std::endl;

    this->stateEstimation->getFeetPosition("fl", &estimated_feet_fl);
    this->stateEstimation->getFeetPosition("fr", &estimated_feet_fr);
    this->stateEstimation->getFeetPosition("bl", &estimated_feet_bl);
    this->stateEstimation->getFeetPosition("br", &estimated_feet_br);


    geometry_msgs::Point p_estimated_feet_fl, p_estimated_feet_fr, p_estimated_feet_bl, p_estimated_feet_br;
    eigen2Point(&p_estimated_feet_fl, &estimated_feet_fl);
    eigen2Point(&p_estimated_feet_fr, &estimated_feet_fr);
    eigen2Point(&p_estimated_feet_bl, &estimated_feet_bl);
    eigen2Point(&p_estimated_feet_br, &estimated_feet_br);

    this->feet_forces_msg.feet_pos[0] = p_estimated_feet_fl;
    this->feet_forces_msg.feet_pos[1] = p_estimated_feet_fr;
    this->feet_forces_msg.feet_pos[2] = p_estimated_feet_bl;
    this->feet_forces_msg.feet_pos[3] = p_estimated_feet_br;

    this->feet_forces_msg.feet_stance = {curr_stance_fl, curr_stance_fr, curr_stance_bl, curr_stance_br};

}


void GaitControl::computePredictiveSupportPolygon(){
   /*********************************************************************************
   * **************************** PREDICTIVE SUPPORT POLYGON **************************
   *********************************************************************************/

    /**************** OBTAIN WEIGHT FACTOR FOR SYPPORT POLYGON  ***************/
    float weight_factor_fl = this->fl_trajectory.getWeightingFactor(this->wfactor_stance_sigma1, this->wfactor_stance_sigma2, this->wfactor_swing_sigma1, this->wfactor_swing_sigma2, this->wfactor_offset);
    float weight_factor_fr = this->fr_trajectory.getWeightingFactor(this->wfactor_stance_sigma1, this->wfactor_stance_sigma2, this->wfactor_swing_sigma1, this->wfactor_swing_sigma2, this->wfactor_offset);
    float weight_factor_bl = this->bl_trajectory.getWeightingFactor(this->wfactor_stance_sigma1, this->wfactor_stance_sigma2, this->wfactor_swing_sigma1, this->wfactor_swing_sigma2, this->wfactor_offset);
    float weight_factor_br = this->br_trajectory.getWeightingFactor(this->wfactor_stance_sigma1, this->wfactor_stance_sigma2, this->wfactor_swing_sigma1, this->wfactor_swing_sigma2, this->wfactor_offset);

    // std::cout << weight_factor_fl << " " << weight_factor_fr << " " << weight_factor_bl << " " << weight_factor_br << std::endl;


    // bool stance_fl, stance_fr, stance_bl, stance_br;
    // stance_fl = this->fl_trajectory.getStance();
    // stance_fr = this->fr_trajectory.getStance();
    // stance_bl = this->bl_trajectory.getStance();
    // stance_br = this->br_trajectory.getStance();
    // std::cout << stance_fl << " " << stance_fr << " " << stance_bl << " " << stance_br << std::endl;

    /**************** SET BASE POSE ***************/
    // std::cout << curr_feet_fl_pos_fromBase.x() << " " << curr_feet_fl_pos_fromBase.y() << " " << curr_feet_fl_pos_fromBase.z() << std::endl;
    // std::cout << curr_feet_fr_pos_fromBase.x() << " " << curr_feet_fr_pos_fromBase.y() << " " << curr_feet_fr_pos_fromBase.z() << std::endl;
    // std::cout << curr_feet_bl_pos_fromBase.x() << " " << curr_feet_bl_pos_fromBase.y() << " " << curr_feet_bl_pos_fromBase.z() << std::endl;
    // std::cout << curr_feet_br_pos_fromBase.x() << " " << curr_feet_br_pos_fromBase.y() << " " << curr_feet_br_pos_fromBase.z() << std::endl << std::endl;


    base_position_offset << this->base_offset_x, this->base_offset_y, 0;

    this->polygonSupport->setLeg("fl", curr_feet_fl_pos_fromBase + base_position_offset, weight_factor_fl);
    this->polygonSupport->setLeg("fr", curr_feet_fr_pos_fromBase + base_position_offset, weight_factor_fr);
    this->polygonSupport->setLeg("bl", curr_feet_bl_pos_fromBase + base_position_offset, weight_factor_bl);
    this->polygonSupport->setLeg("br", curr_feet_br_pos_fromBase + base_position_offset, weight_factor_br);

    this->polygonSupport->getDesiredBodyPos(&psp_body_pos);

    psp_body_pos = base_rotation_matrix * this->factor_predictive_support_polygon * psp_body_pos + estimated_base_pos;
}

void GaitControl::computeFeetDirection(double dt){

    Eigen::Vector3f terrain_norm, terrain_t1, terrain_t2;
    terrain_norm << 0, 0, 1;
    terrain_t1   << 1, 0, 0;
    terrain_t2   << 0, 1, 0;
    float mu      = 5.0;

    /**********************************************************************/
    /********************* FORCE CONTROLLER *******************************/
    /**********************************************************************/

    base_vel_weight.setZero();

    Eigen::Vector3f base_position_offset_rotated;
    base_position_offset_rotated = base_rotation_matrix * base_position_offset;

    geometry_msgs::TwistStamped est_vel_geo, filt_est_vel_geo;

    est_vel_geo.twist.linear.x = estimated_base_vel.x();
    est_vel_geo.twist.linear.y = estimated_base_vel.y();
    est_vel_geo.twist.linear.z = estimated_base_vel.z();

    filter_vel_est.update(&est_vel_geo);


    /*ERROR HERE!

    filter_vel_est.get(&filt_est_vel_geo);
    estimated_base_vel << filt_est_vel_geo.twist.linear.x ,filt_est_vel_geo.twist.linear.y, 0;



    std::cout << "Estimated vel: " << estimated_base_vel.x() << " " << estimated_base_vel.y() << " " << estimated_base_vel.z() << std::endl;

    */


    if (this->enable_force_controller) {



        this->torqController->setFeetForceLimits(MIN_FEET_FORCE, MAX_FEET_FORCE);
        this->torqController->setBaseCoMOffset(base_position_offset_rotated.x(), base_position_offset_rotated.y());

        this->torqController->setTerrainPlane("fl", mu, terrain_norm, terrain_t1, terrain_t2);
        this->torqController->setTerrainPlane("fr", mu, terrain_norm, terrain_t1, terrain_t2);
        this->torqController->setTerrainPlane("bl", mu, terrain_norm, terrain_t1, terrain_t2);
        this->torqController->setTerrainPlane("br", mu, terrain_norm, terrain_t1, terrain_t2);



        this->torqController->setLegState("fl", estimated_feet_fl, curr_stance_fl);
        this->torqController->setLegState("fr", estimated_feet_fr, curr_stance_fr);
        this->torqController->setLegState("bl", estimated_feet_bl, curr_stance_bl);
        this->torqController->setLegState("br", estimated_feet_br, curr_stance_br);


        Eigen::Vector3f zerovec2;
        zerovec2 << 0,0,0;

        this->torqController->setRobotState(estimated_base_pos, zerovec2, base_rotation_matrix, estimated_base_ang_vel);
        this->torqController->setRobotControl(des_base_pos, des_base_vel, des_base_rot, des_base_ang_vel);




        this->torqController->computeForces( dt );

        Eigen::Vector3f des_force_fl, des_force_fr, des_force_bl, des_force_br;
        this->torqController->getForces(&des_force_fl, &des_force_fr, &des_force_bl, &des_force_br);



        des_feet_dir_fl = des_force_fl;
        des_feet_dir_fr = des_force_fr;
        des_feet_dir_bl = des_force_bl;
        des_feet_dir_br = des_force_br;

        this->torqController->getKvel(&base_vel_weight);
    }


    /**************************************************************************/
    /*************************** PREDICTIVE MODEL CONTROLLER *****************/
    /**************************************************************************/
    if (this->enable_pmc_controller){
        double yaw = this->base_euler_angles(2);

        this->predModelController->setTerrainPlane("fl", mu, terrain_norm.cast<double>(), terrain_t1.cast<double>(), terrain_t2.cast<double>(), yaw);
        this->predModelController->setTerrainPlane("fr", mu, terrain_norm.cast<double>(), terrain_t1.cast<double>(), terrain_t2.cast<double>(), yaw);
        this->predModelController->setTerrainPlane("bl", mu, terrain_norm.cast<double>(), terrain_t1.cast<double>(), terrain_t2.cast<double>(), yaw);
        this->predModelController->setTerrainPlane("br", mu, terrain_norm.cast<double>(), terrain_t1.cast<double>(), terrain_t2.cast<double>(), yaw);

        this->predModelController->setFeetPos("fl", estimated_feet_fl.cast<double>());
        this->predModelController->setFeetPos("fr", estimated_feet_fr.cast<double>());
        this->predModelController->setFeetPos("bl", estimated_feet_bl.cast<double>());
        this->predModelController->setFeetPos("br", estimated_feet_br.cast<double>());




        this->predModelController->setBaseCoMOffset(base_position_offset_rotated.x(), base_position_offset_rotated.y());



        // std::cout << "estimated_base_pos    " << estimated_base_pos.x() << " " << estimated_base_pos.y() << " " << estimated_base_pos.z() << std::endl;
        // std::cout << "estimated_base_vel    " << estimated_base_vel.x() << " " << estimated_base_vel.y() << " " << estimated_base_vel.z() << std::endl;
        // std::cout << "estimated_base_ang    " << base_euler_angles.x() << " " << base_euler_angles.y() << " " << base_euler_angles.z() << std::endl;
        // std::cout << "estimated_base_ang_vel" << estimated_base_ang_vel.x() << " " << estimated_base_ang_vel.y() << " " << estimated_base_ang_vel.z() << std::endl;
        // std::cout << " " << std::endl;

        // std::cout << "des_base_vel          " << des_base_vel.x() << " " << des_base_vel.y() << " " << des_base_vel.z() << std::endl;
        // std::cout << "des_base_ang_vel      " << des_base_ang_vel.x() << " " << des_base_ang_vel.y() << " " << des_base_ang_vel.z() << std::endl;

        // std::cout << "----------------------" << std::endl;


        // std::cout << estimated_base_vel.x() << " " << estimated_base_vel.y() << " " << estimated_base_vel.z() << std::endl;


        this->predModelController->setRobotState(estimated_base_pos.cast<double>(), estimated_base_vel.cast<double>(), base_euler_angles.cast<double>(), estimated_base_ang_vel.cast<double>());
        this->predModelController->setRobotControl(des_base_vel.cast<double>(), des_base_ang_vel.cast<double>());

        this->predModelController->computeForces( this->phase );

        Eigen::Vector3d des_feet_dir_fl_d, des_feet_dir_fr_d, des_feet_dir_bl_d, des_feet_dir_br_d, base_vel_weight_d;

        this->predModelController->getForces(&des_feet_dir_fl_d, &des_feet_dir_fr_d, &des_feet_dir_bl_d, &des_feet_dir_br_d);


        this->predModelController->getBaseVelWeight(&base_vel_weight_d);
         des_feet_dir_fl = des_feet_dir_fl_d.cast<float>();
         des_feet_dir_fr = des_feet_dir_fr_d.cast<float>();
         des_feet_dir_bl = des_feet_dir_bl_d.cast<float>();
         des_feet_dir_br = des_feet_dir_br_d.cast<float>();


         // des_feet_dir_fl << -0.2,0,0;
         // des_feet_dir_fr << -0.2,0,0;
         // des_feet_dir_bl << -0.2,0,0;
         // des_feet_dir_br << -0.2,0,0;

         base_vel_weight = base_vel_weight_d.cast<float>();

        /*** Transform for velocity ****/
        // des_feet_dir_fl = des_feet_dir_fl * 4*(1-prop_swing_time)/(ROBOT_TOTAL_MASS);
        // des_feet_dir_fr = des_feet_dir_fr * 4*(1-prop_swing_time)/(ROBOT_TOTAL_MASS);
        // des_feet_dir_bl = des_feet_dir_bl * 4*(1-prop_swing_time)/(ROBOT_TOTAL_MASS);
        // des_feet_dir_br = des_feet_dir_br * 4*(1-prop_swing_time)/(ROBOT_TOTAL_MASS);

    }




    geometry_msgs::Point p_des_force_fl, p_des_force_fr, p_des_force_bl, p_des_force_br;
    eigen2Point(&p_des_force_fl, &des_feet_dir_fl);
    eigen2Point(&p_des_force_fr, &des_feet_dir_fr);
    eigen2Point(&p_des_force_bl, &des_feet_dir_bl);
    eigen2Point(&p_des_force_br, &des_feet_dir_br);

    this->feet_forces_msg.pressure_vector[0] = p_des_force_fl;
    this->feet_forces_msg.pressure_vector[1] = p_des_force_fr;
    this->feet_forces_msg.pressure_vector[2] = p_des_force_bl;
    this->feet_forces_msg.pressure_vector[3] = p_des_force_br;

}

void GaitControl::moveFeet(float dt){
    /**************************************************************************/
    /************************** MAKE STEP *************************************/
    /**************************************************************************/

    // des_feet_dir_fl << 0,0,0;
    // des_feet_dir_fr << 0,0,0;
    // des_feet_dir_bl << 0,0,0;
    // des_feet_dir_br << 0,0,0;

    // des_feet_dir_fl = -des_base_vel_fromBase;
    // des_feet_dir_fr = -des_base_vel_fromBase;
    // des_feet_dir_bl = -des_base_vel_fromBase;
    // des_feet_dir_br = -des_base_vel_fromBase;

    // std::cout << "feet velocity x " <<  des_base_vel_fromBase.x() << std::endl;



    // des_feet_dir_fl = -base_rotation_matrix * des_base_vel_fromBase;
    // des_feet_dir_fr = -base_rotation_matrix * des_base_vel_fromBase;
    // des_feet_dir_bl = -base_rotation_matrix * des_base_vel_fromBase;
    // des_feet_dir_br = -base_rotation_matrix * des_base_vel_fromBase;

    this->fl_trajectory.makeStep(this->contact_fl, this->normalize_stance_time * base_rotation_matrix.transpose() * des_feet_dir_fl, des_base_vel_fromBase, estimated_base_vel_fromBase, this->phase, estimated_base_pos.z(), &feet_fl_pos_cmd);
    this->fr_trajectory.makeStep(this->contact_fr, this->normalize_stance_time * base_rotation_matrix.transpose() * des_feet_dir_fr, des_base_vel_fromBase, estimated_base_vel_fromBase, this->phase, estimated_base_pos.z(), &feet_fr_pos_cmd);
    this->bl_trajectory.makeStep(this->contact_bl, this->normalize_stance_time * base_rotation_matrix.transpose() * des_feet_dir_bl, des_base_vel_fromBase, estimated_base_vel_fromBase, this->phase, estimated_base_pos.z(), &feet_bl_pos_cmd);
    this->br_trajectory.makeStep(this->contact_br, this->normalize_stance_time * base_rotation_matrix.transpose() * des_feet_dir_br, des_base_vel_fromBase, estimated_base_vel_fromBase, this->phase, estimated_base_pos.z(), &feet_br_pos_cmd);


    /******************* OBTAIN JOINT ANGLE COMMANDS ************/


    feet_fl_pos_cmd << feet_fl_pos_cmd.x(), feet_fl_pos_cmd.y(), feet_fl_pos_cmd.z() - this->robot_COM_z_pos;
    feet_fr_pos_cmd << feet_fr_pos_cmd.x(), feet_fr_pos_cmd.y(), feet_fr_pos_cmd.z() - this->robot_COM_z_pos;
    feet_bl_pos_cmd << feet_bl_pos_cmd.x(), feet_bl_pos_cmd.y(), feet_bl_pos_cmd.z() - this->robot_COM_z_pos;
    feet_br_pos_cmd << feet_br_pos_cmd.x(), feet_br_pos_cmd.y(), feet_br_pos_cmd.z() - this->robot_COM_z_pos;


}


void GaitControl::feet_IK(){
    bool hr_fl = this->kinematics->IK_leg_fromRef("fl", feet_fl_pos_cmd, &angles_cmd_fl);
    bool hr_fr = this->kinematics->IK_leg_fromRef("fr", feet_fr_pos_cmd, &angles_cmd_fr);
    bool hr_bl = this->kinematics->IK_leg_fromRef("bl", feet_bl_pos_cmd, &angles_cmd_bl);
    bool hr_br = this->kinematics->IK_leg_fromRef("br", feet_br_pos_cmd, &angles_cmd_br);


    /******************* SEND JOINT COMMANDS ************/
    if((hr_fl)&&(hr_fr)&&(hr_bl)&&(hr_br)) {    // if IK Reachability is possible



        this->joint_control_msg.position = {angles_cmd_fl.x(), angles_cmd_fl.y(), angles_cmd_fl.z(),
                                            angles_cmd_fr.x(), angles_cmd_fr.y(), angles_cmd_fr.z(),
                                            angles_cmd_bl.x(), angles_cmd_bl.y(), angles_cmd_bl.z(),
                                            angles_cmd_br.x(), angles_cmd_br.y(), angles_cmd_br.z()};

        // If the position was ok, then update the feet trajectory current position
        this->fl_trajectory.updateFeePos();
        this->fr_trajectory.updateFeePos();
        this->bl_trajectory.updateFeePos();
        this->br_trajectory.updateFeePos();


    }

}

/************ main loop ****************/
void GaitControl::mainNodeThread(){
    if (performing_sequence){
        return;
    }


    std::cout << std::setprecision(3) << std::fixed;
    float dt = (ros::Time::now() - this->dtTimer).toSec();
    this->dtTimer = ros::Time::now();

    this->computeRobotState(dt);
    this->computePredictiveSupportPolygon();


    this->robot_COM_z_pos = this->robot_COM_z_pos + this->vel_cmd.twist.linear.z * dt;
    des_base_vel << this->vel_cmd.twist.linear.x, this->vel_cmd.twist.linear.y, this->vel_cmd.twist.linear.z;
    des_base_vel = base_rotation_matrix * des_base_vel;

    // When the robot is not walking, the 3 angles can be controlled and the position of the base is locked
    des_base_ang_vel << this->vel_cmd.twist.angular.x, this->vel_cmd.twist.angular.y, this->vel_cmd.twist.angular.z;
    des_base_pos     << locked_base_pos.x(), locked_base_pos.y(), this->robot_COM_z_pos  + des_base_vel.z() * dt;

    // when the robot walks, the desired base pos is incremented with the velocity and only the yaw angle can be controlled through angular velocity commands
    if (this->walk){
         des_base_pos << psp_body_pos.x() + des_base_vel.x() * dt,
                        psp_body_pos.y() + des_base_vel.y() * dt,
                        this->robot_COM_z_pos + des_base_vel.z() * dt;

         des_base_ang_vel << 0,0,des_base_ang_vel.z();

        if ((des_base_vel.norm() > this->linear_vel_th_to_walk)|| (des_base_ang_vel.norm() > linear_ang_vel_th_to_walk)) {
            this->fl_trajectory.setWalking(true);
            this->fr_trajectory.setWalking(true);
            this->bl_trajectory.setWalking(true);
            this->br_trajectory.setWalking(true);
        }
        else{
            des_base_vel     << 0,0,0;
            des_base_ang_vel << 0,0,0;
            this->fl_trajectory.setWalking(false);
            this->fr_trajectory.setWalking(false);
            this->bl_trajectory.setWalking(false);
            this->br_trajectory.setWalking(false);
            // this->initPhase();

        }
    }


    des_base_ang_vel = base_rotation_matrix * des_base_ang_vel;




    /*************** PROTECT ROBOT WHEN FALLS *****************/

    if (!this->motors_disconnected){
        if (  (std::abs(this->base_euler_angles.x()) > MAX_ROLL_TO_DISCONNECT_MOTORS) || (std::abs(this->base_euler_angles.y()) > MAX_PITCH_TO_DISCONNECT_MOTORS) ){
            this->setGaitFromFile("stop");
            this->motors_disconnected = true;


            catbot_msgs::connect_servos connectServos_msg;
            connectServos_msg.request.connect = false;
            if (this->connectMotorsClient.call(connectServos_msg)){
                ROS_INFO("Servos desconnected");
            }
          else{
            ROS_ERROR("Failed to call service connect_servos");
            }
        }
    }





    des_feet_dir_fl.setZero();
    des_feet_dir_fr.setZero();
    des_feet_dir_bl.setZero();
    des_feet_dir_br.setZero();


    this->computeFeetDirection(dt);


    Eigen::Vector3f vel_cmd_normalized;
    vel_cmd_normalized << des_base_vel.x() / this->max_abs_vel_x, des_base_vel.y() / this->max_abs_vel_y, des_base_ang_vel.z() / this->max_abs_vel_yaw;
    float vel_factor = vel_cmd_normalized.norm();
    if (vel_factor > 1) vel_factor = 1;

    this->gait_period = this->gait_period_min_lim; //this->gait_period_max_lim - vel_factor*(this-> gait_period_max_lim - this->gait_period_min_lim);
    this->getPhase(this->gait_period);


    float swing_time = this->prop_swing_time*this->gait_period_min_lim;


    float prop_swing_time_t = swing_time / this->gait_period;

    this->fl_trajectory.setPeriodAndSwing(this->gait_period, prop_swing_time_t);
    this->fr_trajectory.setPeriodAndSwing(this->gait_period, prop_swing_time_t);
    this->bl_trajectory.setPeriodAndSwing(this->gait_period, prop_swing_time_t);
    this->br_trajectory.setPeriodAndSwing(this->gait_period, prop_swing_time_t);


    float stance_time_min       = (1 - this->prop_swing_time)*this->gait_period_min_lim;
    float stance_time_t         =  (1 - prop_swing_time_t)*this->gait_period;
    this->normalize_stance_time = stance_time_min/stance_time_t;

    // std::cout << vel_factor << " " << prop_swing_time_t  << " " <<  this->normalize_stance_time  << " " << this->gait_period << std::endl;

    // if (des_feet_dir_fl.x() > 0){
        // std::cout << "feet pos fl "  << des_feet_dir_fl.x() << " " << des_feet_dir_fl.y() << " " << des_feet_dir_fl.z() << std::endl << std::endl;
    // }

    // Eigen::Vector3f feet_fl_pos, feet_fr_pos, feet_bl_pos, feet_br_pos, desired_base_vel, desired_body_pos, body_pos, body_rpy;

    /**************** GET BODY VELOCITY COMMAND ************************/
    // desired_base_vel << 0,0,0;
    /*
    if (this->walk) {
    // desired_base_vel << 0.0 + this->vel_cmd.twist.linear.x, this->vel_cmd.twist.linear.y, 0;
    }
    else{
        if (estimated_base_vel.norm() >= MIN_BASE_VELOCITY_FOR_REACTION){
            this->initPhase();
            this->fl_trajectory.setWalking(true);
            this->fr_trajectory.setWalking(true);
            this->bl_trajectory.setWalking(true);
            this->br_trajectory.setWalking(true);
        }
        else{
            this->fl_trajectory.setWalking(false);
            this->fr_trajectory.setWalking(false);
            this->bl_trajectory.setWalking(false);
            this->br_trajectory.setWalking(false);
        }
    }
    */
    // std::cout << this->phase << std::endl;
    /**************** GET FEET POSITON COMMAND (FROM REF) ***************/

    // std::cout << this->robot_state.linear_velocity.x << " " << this->robot_state.linear_velocity.y << " " << this->robot_state.linear_velocity.z << std::endl;


    Eigen::Vector3f des_base_vel_fromBase_; //, des_base_vel_fromBase, estimated_base_vel_fromBase;

    des_base_vel_fromBase_ =  base_rotation_matrix.transpose() * des_base_vel ;


    des_base_vel_fromBase <<  des_base_vel_fromBase_.x(),// * base_vel_weight.x(),
                              des_base_vel_fromBase_.y(),// * base_vel_weight.y(),
                              des_base_vel_fromBase_.z();// * base_vel_weight.z();
     // std::cout << des_base_vel_fromBase.x() << " " << des_base_vel_fromBase.y() << " " << des_base_vel_fromBase.z() << std::endl;
     // std::cout << base_vel_weight.x() << " " << base_vel_weight.y() << " " << base_vel_weight.z() << std::endl;

    // std::cout << "-------------" << std::endl;



    estimated_base_vel_fromBase =  base_rotation_matrix.transpose() * estimated_base_vel ;

    this->moveFeet(dt);
    this->feet_IK();


    this->publishData();




    // this->loop_counter += 1;


}



void GaitControl::publishData(){
    double dt = (ros::Time::now() - this->publishControlTimer).toSec();
    if (dt > 1.0/PUBLISH_FREQ_CONTROL){

        this->joint_control_msg.header.stamp = ros::Time::now();
        this->jointControlPub.publish(this->joint_control_msg);
        this->publishControlTimer = ros::Time::now();
    }

    double dt2 = (ros::Time::now() - this->publishVisTimer).toSec();
    if (dt2 > 1.0/PUBLISH_FREQ_VIS){

        this->feet_press.header.stamp = ros::Time::now();
        this->feetPressurePub.publish(this->feet_press);

        this->feet_forces_msg.header.stamp = ros::Time::now();
        this->feetForceControlPub.publish(this->feet_forces_msg);


        Eigen::Vector3f estimated_base_pos;
        this->stateEstimation->getBasePosition(&estimated_base_pos);

        // std::cout << estimated_base_pos.x() << " " << estimated_base_pos.y() << " " << estimated_base_pos.z() << std::endl;
        // std::cout << "_________________" << std::endl;


        tf::Vector3 robot_pos(estimated_base_pos.x(), estimated_base_pos.y(), estimated_base_pos.z());
        tf::Quaternion robot_q( this->imu_data.orientation.x, this->imu_data.orientation.y, this->imu_data.orientation.z, this->imu_data.orientation.w );

        tf::Transform robot_tf;
        robot_tf.setOrigin( robot_pos );
        robot_tf.setRotation( robot_q );


        // this->tf_br.sendTransform(tf::StampedTransform(robot_tf, this->imu_data.header.stamp, "world", "base_link"));
        this->tf_br.sendTransform(tf::StampedTransform(robot_tf, ros::Time::now(), "odom", "base_link"));
        this->publishVisTimer = ros::Time::now();

        this->odometry_msg.header.stamp = ros::Time::now();
        this->odometry_msg.header.frame_id = "odom";
        this->odometry_msg.child_frame_id  = "base_link";
        this->odometry_msg.pose.pose.position.x = estimated_base_pos.x();
        this->odometry_msg.pose.pose.position.y = estimated_base_pos.y();
        this->odometry_msg.pose.pose.position.z = estimated_base_pos.z();

        this->odometry_msg.pose.pose.orientation.x = robot_q.x();
        this->odometry_msg.pose.pose.orientation.y = robot_q.y();
        this->odometry_msg.pose.pose.orientation.z = robot_q.z();
        this->odometry_msg.pose.pose.orientation.w = robot_q.w();


        this->odometry_msg.twist.twist.linear.x = this->estimated_base_vel.x();
        this->odometry_msg.twist.twist.linear.y = this->estimated_base_vel.y();
        this->odometry_msg.twist.twist.linear.z = this->estimated_base_vel.z();

        this->odometry_msg.twist.twist.angular.x = this->estimated_base_ang_vel.x();
        this->odometry_msg.twist.twist.angular.y = this->estimated_base_ang_vel.y();
        this->odometry_msg.twist.twist.angular.z = this->estimated_base_ang_vel.z();


        this->odometryPub.publish(this->odometry_msg);
    }


}




void GaitControl::setGaitFromFile(std::string file){

    std::string file_dir = this->gaits_dir + std::string("/") + file + std::string(".gait");

    std::ifstream infile(file_dir);
    if(!infile.good()){
        ROS_ERROR("Gait file was not found, remember not to include extension '.gait' in the name");
        return;
    }

    ConfigFile cf(file_dir);

    bool walk_ = cf.Value("gait settings", "walk");
    this->motors_disconnected = false;

    // if the file sets the robot to not walk, lock the position of the robot and the angles:
    if (!walk_){
        this->locked_base_pos = estimated_base_pos;

        Eigen::AngleAxisd yawAngle(base_euler_angles.z(), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

        des_base_rot = q.matrix().cast<float>();
    }

    // if ((walk_)||(!this->set_initial_gait_file)) {


        this->gait_period_min_lim = cf.Value("gait settings", "gait_period_min");
        this->gait_period_max_lim = cf.Value("gait settings", "gait_period_max");
        this->gait_period = this->gait_period_max_lim;

        this->prop_swing_time = cf.Value("gait settings", "prop_swing_time");
        float fl_phase_offset = cf.Value("gait settings", "fl_phase_offset");
        float fr_phase_offset = cf.Value("gait settings", "fr_phase_offset");
        float bl_phase_offset = cf.Value("gait settings", "bl_phase_offset");
        float br_phase_offset = cf.Value("gait settings", "br_phase_offset");

        float feet_offset_x = cf.Value("Feet and base settings", "feet_offset_x");
        float feet_offset_y = cf.Value("Feet and base settings", "feet_offset_y");
        this->base_offset_x = cf.Value("Feet and base settings", "base_offset_x");
        this->base_offset_y = cf.Value("Feet and base settings", "base_offset_y");
        float step_height = cf.Value("Feet and base settings", "step_height");
        this->force_contact_threshold = cf.Value("Feet and base settings", "force_contact_threshold");
        float capture_point_velocity_factor = cf.Value("Feet and base settings", "capture_point_velocity_factor");


        this->fl_trajectory.setPeriodAndSwing(this->gait_period, this->prop_swing_time);
        this->fr_trajectory.setPeriodAndSwing(this->gait_period, this->prop_swing_time);
        this->bl_trajectory.setPeriodAndSwing(this->gait_period, this->prop_swing_time);
        this->br_trajectory.setPeriodAndSwing(this->gait_period, this->prop_swing_time);

        this->fl_trajectory.setTrajectoryParams(fl_phase_offset, step_height, feet_offset_x, feet_offset_y, this->base_offset_x, this->base_offset_y, capture_point_velocity_factor);
        this->fr_trajectory.setTrajectoryParams(fr_phase_offset, step_height, feet_offset_x, feet_offset_y, this->base_offset_x, this->base_offset_y, capture_point_velocity_factor);
        this->bl_trajectory.setTrajectoryParams(bl_phase_offset, step_height, feet_offset_x, feet_offset_y, this->base_offset_x, this->base_offset_y, capture_point_velocity_factor);
        this->br_trajectory.setTrajectoryParams(br_phase_offset, step_height, feet_offset_x, feet_offset_y, this->base_offset_x, this->base_offset_y, capture_point_velocity_factor);



        float min_prop_swing_time_before_contact_detection = cf.Value("Feet and base settings", "min_prop_swing_time_before_contact_detection");
        float dt_contact_detection_seconds                 = cf.Value("Feet and base settings", "dt_contact_detection_seconds");

        this->fl_trajectory.setFootContactParams(min_prop_swing_time_before_contact_detection, dt_contact_detection_seconds);
        this->fr_trajectory.setFootContactParams(min_prop_swing_time_before_contact_detection, dt_contact_detection_seconds);
        this->bl_trajectory.setFootContactParams(min_prop_swing_time_before_contact_detection, dt_contact_detection_seconds);
        this->br_trajectory.setFootContactParams(min_prop_swing_time_before_contact_detection, dt_contact_detection_seconds);


        this->factor_predictive_support_polygon = cf.Value("Predictive support polygon settings",
                                                           "factor_predictive_support_polygon");
        this->wfactor_stance_sigma1 = cf.Value("Predictive support polygon settings", "wfactor_stance_sigma1");
        this->wfactor_stance_sigma2 = cf.Value("Predictive support polygon settings", "wfactor_stance_sigma2");
        this->wfactor_swing_sigma1 = cf.Value("Predictive support polygon settings", "wfactor_swing_sigma1");
        this->wfactor_swing_sigma2 = cf.Value("Predictive support polygon settings", "wfactor_swing_sigma2");
        this->wfactor_offset = cf.Value("Predictive support polygon settings", "wfactor_offset");


        this->max_abs_vel_x = cf.Value("Control settings", "max_abs_vel_x");
        this->max_abs_vel_y= cf.Value("Control settings", "max_abs_vel_y");
        this->max_abs_vel_z = cf.Value("Control settings", "max_abs_vel_z");
        this->max_abs_vel_roll  = cf.Value("Control settings", "max_abs_vel_roll");
        this->max_abs_vel_pitch = cf.Value("Control settings", "max_abs_vel_pitch");
        this->max_abs_vel_yaw   = cf.Value("Control settings", "max_abs_vel_yaw");
        this->filter_vel_cmd_factor = cf.Value("Control settings", "filter_vel_cmd_factor");
        float  filter_estimated_vel_factor = cf.Value("Control settings", "filter_estimated_vel_factor");

        this->filter_vel_cmd.setFactor(filter_vel_cmd_factor);
        this->filter_vel_est.setFactor(filter_estimated_vel_factor);

        this->linear_vel_th_to_walk = cf.Value("Control settings", "linear_vel_th_to_walk");
        this->linear_ang_vel_th_to_walk = cf.Value("Control settings", "linear_ang_vel_th_to_walk");



        this->enable_force_controller = cf.Value("Force controller settings", "enable_force_controller");
        if (this->enable_force_controller){
            this->fl_trajectory.useIntegratedPos(false);
            this->fr_trajectory.useIntegratedPos(false);
            this->bl_trajectory.useIntegratedPos(false);
            this->br_trajectory.useIntegratedPos(false);
        }
        float k_pos_x = cf.Value("Force controller settings", "force_controller_k_base_position_x");
        float k_pos_y = cf.Value("Force controller settings", "force_controller_k_base_position_y");
        float k_pos_z = cf.Value("Force controller settings", "force_controller_k_base_position_z");

        float k_vel_x = cf.Value("Force controller settings", "force_controller_k_base_velocity_x");
        float k_vel_y = cf.Value("Force controller settings", "force_controller_k_base_velocity_y");
        float k_vel_z = cf.Value("Force controller settings", "force_controller_k_base_velocity_z");

        float k_rot_x = cf.Value("Force controller settings", "force_controller_k_base_angles_x");
        float k_rot_y = cf.Value("Force controller settings", "force_controller_k_base_angles_y");
        float k_rot_z = cf.Value("Force controller settings", "force_controller_k_base_angles_z");

        float k_ang_vel_x = cf.Value("Force controller settings", "force_controller_k_base_angular_vel_x");
        float k_ang_vel_y = cf.Value("Force controller settings", "force_controller_k_base_angular_vel_y");
        float k_ang_vel_z = cf.Value("Force controller settings", "force_controller_k_base_angular_vel_z");

        float w_dyn_model = cf.Value("Force controller settings", "force_controller_dynamical_model_weight");
        float w_forces = cf.Value("Force controller settings", "force_controller_regularization_weight");
        float w_prev_forces = cf.Value("Force controller settings", "force_controller_prev_force_weight");
        float force_controller_min_force = cf.Value("Force controller settings", "force_controller_min_force");
        float force_controller_max_force = cf.Value("Force controller settings", "force_controller_max_force");

        Eigen::Vector3f k_pos_, k_vel_, k_rot_, k_ang_vel_;
        k_pos_ <<  k_pos_x, k_pos_y, k_pos_z;
        k_vel_ <<  k_vel_x, k_vel_y, k_vel_z;
        k_rot_ <<  k_rot_x, k_rot_y, k_rot_z;
        k_ang_vel_ <<  k_ang_vel_x, k_ang_vel_y, k_ang_vel_z;

        this->torqController->setProportionalConstController(k_pos_, k_vel_, k_rot_, k_ang_vel_);
        this->torqController->setWeightsController(w_dyn_model, w_forces, w_prev_forces);
        this->torqController->setFeetForceLimits(force_controller_min_force, force_controller_max_force);



        /******************* PMC ***********************/
        this->enable_pmc_controller = cf.Value("Predictive Model controller settings", "enable_pmc_controller");
        if (this->enable_pmc_controller){
            this->fl_trajectory.useIntegratedPos(true);
            this->fr_trajectory.useIntegratedPos(true);
            this->bl_trajectory.useIntegratedPos(true);
            this->br_trajectory.useIntegratedPos(true);
        }

        float pcm_w_forces = cf.Value("Predictive Model controller settings", "pmc_controller_w_forces");


        this->predModelController->setHorizon_and_dtime( cf.Value("Predictive Model controller settings", "horizon"),
                                                         cf.Value("Predictive Model controller settings", "deltat"));


        Eigen::Vector3f pmc_base_pos_weight, pmc_base_ang_weight, pmc_base_vel_weight, pmc_base_ang_vel_weight;
        pmc_base_pos_weight     <<  cf.Value("Predictive Model controller settings", "pmc_controller_w_base_position_x"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_position_y"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_position_z");

        pmc_base_ang_weight     <<  cf.Value("Predictive Model controller settings", "pmc_controller_w_base_angles_x"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_angles_y"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_angles_z");

        pmc_base_vel_weight     <<  cf.Value("Predictive Model controller settings", "pmc_controller_w_base_vel_x"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_vel_y"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_vel_z");

        pmc_base_ang_vel_weight <<  cf.Value("Predictive Model controller settings", "pmc_controller_w_base_ang_vel_x"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_ang_vel_y"),
                                    cf.Value("Predictive Model controller settings", "pmc_controller_w_base_ang_vel_z");

        this->predModelController->setWeightsController(pmc_base_pos_weight.cast<double>(),
                                                        pmc_base_ang_weight.cast<double>(),
                                                        pmc_base_vel_weight.cast<double>(),
                                                        pmc_base_ang_vel_weight.cast<double>(),
                                                        pcm_w_forces);

        float pmc_min_force = cf.Value("Predictive Model controller settings", "pmc_controller_min_force");
        float pmc_max_force = cf.Value("Predictive Model controller settings", "pmc_controller_max_force");


        this->predModelController->setFeetForceLimits(pmc_min_force, pmc_max_force);

        float pmc_max_feet_vel = cf.Value("Predictive Model controller settings", "pmc_max_feet_vel");
        this->predModelController->setMaxFeetVelocity(pmc_max_feet_vel);

    // }


    if (this->walk != walk_) {
        this->walk = walk_;
        if (this->walk) {
            this->initPhase();
            this->fl_trajectory.setWalking(true);
            this->fr_trajectory.setWalking(true);
            this->bl_trajectory.setWalking(true);
            this->br_trajectory.setWalking(true);
        }
        else{
            this->fl_trajectory.setWalking(false);
            this->fr_trajectory.setWalking(false);
            this->bl_trajectory.setWalking(false);
            this->br_trajectory.setWalking(false);
        }

    }

    if (!this->set_initial_gait_file){
        this->walk = walk_;
        this->initPhase();
        this->fl_trajectory.setWalking(walk_);
        this->fr_trajectory.setWalking(walk_);
        this->bl_trajectory.setWalking(walk_);
        this->br_trajectory.setWalking(walk_);
        this->set_initial_gait_file = true;
    }

    std::cout << "setting gait: " << file << std::endl;


}


void GaitControl::setSequenceFromFile(std::string file){
    this->setGaitFromFile("sequence");
    this->setGaitFromFile("inplace");

    this->fl_trajectory.setWalkingState(false);
    this->fr_trajectory.setWalkingState(false);
    this->bl_trajectory.setWalkingState(false);
    this->br_trajectory.setWalkingState(false);


    std::string file_dir = this->sequences_dir + std::string("/") + file + std::string(".sequence");

    std::ifstream infile(file_dir);
    if(!infile.good()){
        ROS_ERROR("Gait file was not found, remember not to include extension '.sequence' in the name");
        return;
    }

    ConfigFile cf(file_dir);



    /********* get number of positions in sequence *********/
    std::string position_i;
    int num_positions = 0;
    while(true){
        position_i = std::string("position ") + std::to_string(num_positions);
        try{
            float base_offset_x_ = cf.Value(position_i, "base_pos_x");
        }
        catch (...){
            break;
        }
        num_positions += 1;
    }
    std::cout << "num positions " << num_positions << std::endl;

    /**********************************************************/


    double dt;
    float time_to_complete;
    ros::Time positionTimer;

    Eigen::Vector3f initial_base_pos, final_base_pos, vel_base_cmd;
    Eigen::Vector3f initial_base_angs, final_base_angs, ang_vel_base_cmd_rot, ang_vel_base_cmd;
    tf::Matrix3x3 base_rotation_matrix_tf;
    Eigen::Vector3f des_base_euler_angs_cmd;


    final_base_pos << 0,0,estimated_base_pos.z();


    for(int i=0; i < num_positions; i++){
        position_i = std::string("position ") + std::to_string(i);
        std::cout << position_i << std::endl;

        time_to_complete = cf.Value(position_i, "time_to_complete");



        positionTimer = ros::Time::now();

        Eigen::Vector3f estimated_base_pos;

        this->computeRobotState(0.0001);
        this->stateEstimation->getBasePosition(&estimated_base_pos);

        initial_base_pos  = estimated_base_pos;


        final_base_pos << cf.Value(position_i, "base_pos_x"), cf.Value(position_i, "base_pos_y"), cf.Value(position_i, "base_pos_z");
        final_base_pos = initial_base_pos + base_rotation_matrix * final_base_pos;
        vel_base_cmd = (final_base_pos - initial_base_pos)/time_to_complete;

        initial_base_angs = base_euler_angles;
        final_base_angs << cf.Value(position_i, "base_angle_roll"), cf.Value(position_i, "base_angle_pitch"), initial_base_angs.z() + cf.Value(position_i, "base_angle_yaw");
        ang_vel_base_cmd =  (final_base_angs - initial_base_angs)/time_to_complete;
        ang_vel_base_cmd_rot = base_rotation_matrix * ang_vel_base_cmd;




        int num_stance_legs = curr_stance_fl + curr_stance_fr + curr_stance_bl + curr_stance_br;



        /************ GET FINAL FEET POS (FROM REF) **************/
        Eigen::Vector3f final_pos_fl, final_pos_fr, final_pos_bl, final_pos_br;
        final_pos_fl << cf.Value(position_i, "feet_pos_fl_x"), cf.Value(position_i, "feet_pos_fl_y"), cf.Value(position_i, "feet_pos_fl_z");
        final_pos_fr << cf.Value(position_i, "feet_pos_fr_x"), cf.Value(position_i, "feet_pos_fr_y"), cf.Value(position_i, "feet_pos_fr_z");
        final_pos_bl << cf.Value(position_i, "feet_pos_bl_x"), cf.Value(position_i, "feet_pos_bl_y"), cf.Value(position_i, "feet_pos_bl_z");
        final_pos_br << cf.Value(position_i, "feet_pos_br_x"), cf.Value(position_i, "feet_pos_br_y"), cf.Value(position_i, "feet_pos_br_z");

        /************ GET CURRENT ANGLES **************/
        Eigen::Vector3f initial_angles_fl, initial_angles_fr, initial_angles_bl, initial_angles_br;
        this->getLegStateAngles("fl", &initial_angles_fl);
        this->getLegStateAngles("fr", &initial_angles_fr);
        this->getLegStateAngles("bl", &initial_angles_bl);
        this->getLegStateAngles("br", &initial_angles_br);

        /************ GET INITIAL FEET POS (FROM REF) **************/
        Eigen::Vector3f initial_pos_fl, initial_pos_fr, initial_pos_bl, initial_pos_br;
        this->kinematics->feetPosFromRef("fl", initial_angles_fl, &initial_pos_fl);
        this->kinematics->feetPosFromRef("fr", initial_angles_fr, &initial_pos_fr);
        this->kinematics->feetPosFromRef("bl", initial_angles_bl, &initial_pos_bl);
        this->kinematics->feetPosFromRef("br", initial_angles_br, &initial_pos_br);

        /************ GET VELOCITY TO MOVE FEET *******************/
        Eigen::Vector3f vel_fl, vel_fr, vel_bl, vel_br;
        vel_fl << (final_pos_fl - initial_pos_fl)/time_to_complete;
        vel_fr << (final_pos_fr - initial_pos_fr)/time_to_complete;
        vel_bl << (final_pos_bl - initial_pos_bl)/time_to_complete;
        vel_br << (final_pos_br - initial_pos_br)/time_to_complete;


        int stance_feet_num;


        while(true){


            dt = (ros::Time::now() - positionTimer).toSec();
            if (dt > time_to_complete){
                break;
            }
            // std::cout << dt << std::endl;

            this->computeRobotState(dt);
            this->computePredictiveSupportPolygon();
            // std::cout << "walk " << this->walk << std::endl;
            curr_stance_fl = cf.Value(position_i, "stance_fl");
            curr_stance_fr = cf.Value(position_i, "stance_fr");
            curr_stance_bl = cf.Value(position_i, "stance_bl");
            curr_stance_br = cf.Value(position_i, "stance_br");

            stance_feet_num = curr_stance_fl + curr_stance_fr + curr_stance_bl + curr_stance_br;

            des_base_vel << vel_base_cmd;
            des_base_pos = (initial_base_pos + des_base_vel * dt );


            des_base_ang_vel = base_rotation_matrix * ang_vel_base_cmd;

            des_base_euler_angs_cmd = initial_base_angs + ang_vel_base_cmd * dt;

            // des_base_euler_angs_cmd << initial_base_angs.x() +  ang_vel_base_cmd.x() * dt,
            // ang_vel_base_cmd.y() * dt, // yaw does accumulate, not absolute
            // initial_base_angs.z() + ang_vel_base_cmd.z() * dt;


            // this->kinematics->getRotationMatrix(des_base_euler_angs_cmd, &des_base_rot);
            Eigen::AngleAxisd rollAngle(des_base_euler_angs_cmd.z(), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd yawAngle(des_base_euler_angs_cmd.y(), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd pitchAngle(des_base_euler_angs_cmd.x(), Eigen::Vector3d::UnitX());
            Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
            des_base_rot = q.matrix().cast<float>();




            if (stance_feet_num > 0){
                this->computeFeetDirection(dt);
                this->moveFeet(dt);

            }


            if (!curr_stance_fl){
                feet_fl_pos_cmd = initial_pos_fl + vel_fl * dt;
            }
            if (!curr_stance_fr){
                feet_fr_pos_cmd = initial_pos_fr + vel_fr * dt;
            }
            if (!curr_stance_bl){
                feet_bl_pos_cmd = initial_pos_bl + vel_bl * dt;;
            }
            if (!curr_stance_br){
                feet_br_pos_cmd = initial_pos_br + vel_br * dt;;
            }

            this->feet_IK();
            this->publishData();
            ros::spinOnce();


            // this->base_offset_y = cf.Value(position_i, "base_offset_y");
            /*
            if (num_stance_legs > 0){

                this-> vel_cmd.twist.linear.x = vel_base_x;
                this-> vel_cmd.twist.linear.y = vel_base_y;
                this-> vel_cmd.twist.linear.z = vel_base_z;


                this->mainNodeThread();
            }
            */



            /*


            hr_fl = this->kinematics->IK_leg_fromRef("fl", curr_pos_fl, &curr_angles_fl);
            hr_fr = this->kinematics->IK_leg_fromRef("fr", curr_pos_fr, &curr_angles_fr);
            hr_bl = this->kinematics->IK_leg_fromRef("bl", curr_pos_bl, &curr_angles_bl);
            hr_br = this->kinematics->IK_leg_fromRef("br", curr_pos_br, &curr_angles_br);




            if((hr_fl)&&(hr_fr)&&(hr_bl)&&(hr_br)) {    // if IK Reachability is possible



                this->joint_control_msg.position = {curr_angles_fl.x(), curr_angles_fl.y(), curr_angles_fl.z(),
                                                    curr_angles_fr.x(), curr_angles_fr.y(), curr_angles_fr.z(),
                                                    curr_angles_bl.x(), curr_angles_bl.y(), curr_angles_bl.z(),
                                                    curr_angles_br.x(), curr_angles_br.y(), curr_angles_br.z()};


            }

            this->publishData();
            ros::spinOnce();
            */


        }


        std::cout << "completed time position " << i << std::endl;


    }

    std::cout << "SET SEQUENCE: " << file << std::endl;

}



void GaitControl::debug(std::string type, std::string msg){
	std::cout << type << ": " << msg << std::endl;
}


int main(int argc, char** argv){
	ros::init(argc, argv,"gait_control_node");
	ros::NodeHandle* nh = new ros::NodeHandle();
	ros::Rate loop_rate(100);

	std::string urdf_path, legs_format, robot_name;
    legs_format = "forward";
    robot_name = "catbot";
    ros::param::get("/robot_name", robot_name);

    ros::param::get("/urdf_path", urdf_path);
    ros::param::get("/legs_format", legs_format);

    std::string sequences_dir = ros::package::getPath("gait_control") + std::string("/sequences/");
    std::string gaits_dir     = ros::package::getPath("gait_control") + std::string("/gaits/");
    ros::param::get("/sequences_dir", sequences_dir);
    ros::param::get("/gaits_dir",     gaits_dir);

	GaitControl node(nh, robot_name, urdf_path, legs_format, gaits_dir, sequences_dir);


	while (ros::ok()){
		node.mainNodeThread();
		ros::spinOnce();
		// loop_rate.sleep();  //comment this line for fast!
	}
	return 0;
}
