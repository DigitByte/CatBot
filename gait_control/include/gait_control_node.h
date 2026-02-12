#ifndef _gait_control_node_
#define _gait_control_node_


#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono>
#include <fstream>

#include "catbot_msgs/robot_state.h"
#include "catbot_msgs/set_gait.h"
#include "catbot_msgs/connect_servos.h"

#include "catbot_msgs/feet_pressure.h"
#include "catbot_msgs/contact_detection.h"
#include "sensor_msgs/Imu.h"
#include "feet_trajectory.h"
#include "kinematics.h"
#include "feet_trajectory.h"
#include "polygon_support.h"
#include "torque_controller.h"
#include "filterSmooth.h"
#include "predictive_model_control.h"
#include "contact_detection.h"
#include "state_estimation.h"
#include "ConfigFile.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include "settings.h"
#include <tf/tf.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief Specific Algorithm Class
 *
 */
class GaitControl
{
  private:

	ros::NodeHandle* nh;
    
    // [publisher attributes]
	ros::Publisher jointControlPub;
    ros::Publisher feetPressurePub;
    ros::Publisher feetForceControlPub;
    ros::Publisher odometryPub;
    // [subscriber attributes]
	ros::Subscriber robotStateSub;
    ros::Subscriber torsoControlSub;
    ros::Subscriber velCmdSub;
    ros::Subscriber jointStateSub;
    ros::Subscriber imuDataSub;
    ros::Subscriber contactDetectionSub;



    // [service attributes]
    ros::ServiceServer setGaitService;
    ros::ServiceServer setSequenceService;

    // [client attributes]
    ros::ServiceClient connectMotorsClient;
    // [action server attributes]

    // [action client attributes]

    // [transforms]
	// tf::TransformBroadcaster broadcaster;
		

    /********** private methods ************/

	void debug(std::string type, std::string msg);

	void initPhase();
    void getPhase(double period);
    void publishData();
    void setGaitFromFile(std::string file);
    void setSequenceFromFile(std::string file);

    /************ callbacks ************/
	void robotStateCB(const catbot_msgs::robot_state::ConstPtr &msg);
    void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
    void contactDetectionCB(const catbot_msgs::contact_detection::ConstPtr &msg);

    void imuDataCB(const sensor_msgs::Imu::ConstPtr &msg);
    void velCmdCB(const geometry_msgs::Twist::ConstPtr &msg);
    void torsoControlCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void getLegStateAngles (std::string leg, Eigen::Vector3f* angles);
    void getLegStateTorques(std::string leg, Eigen::Vector3f* torques);
    bool setGaitCB(catbot_msgs::set_gait::Request &req, catbot_msgs::set_gait::Response &res);
    bool setSequenceCB(catbot_msgs::set_gait::Request &req, catbot_msgs::set_gait::Response &res);


    void computeRobotState(float dt);
    void computePredictiveSupportPolygon();
    void computeFeetDirection(double dt);
    void moveFeet(float dt);
    void feet_IK();
public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
	GaitControl(ros::NodeHandle* n, std::string robot_name, std::string urdf_path, std::string legs_format, std::string gaits_dir, std::string sequences_dir);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~GaitControl();

  public:
   /**
    * \brief main node thread
    *
    * This is the main loop node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread();


private:
    tf::TransformBroadcaster tf_br;
    std::string gaits_dir;
    std::string sequences_dir;
    std::string robot_name;

    nav_msgs::Odometry odometry_msg;

    catbot_msgs::robot_state robot_state;
    sensor_msgs::JointState       joint_state;
    geometry_msgs::TwistStamped   vel_cmd;
    geometry_msgs::PoseStamped    body_pose_cmd;
    geometry_msgs::PoseStamped    body_pose;
    sensor_msgs::JointState       joint_control_msg;
    sensor_msgs::Imu              imu_data;
    catbot_msgs::contact_detection contact_detection_msg;
    catbot_msgs::feet_pressure feet_press;
    catbot_msgs::feet_pressure feet_forces_msg;

    ros::Time phaseTimer;
    ros::Time publishControlTimer;
    ros::Time publishVisTimer;

    ros::Time dtTimer;
    ros::Time dtTimer_IMU;


    unsigned long phase_counter;
    double phase;
    bool performing_sequence;

    Kinematics*                kinematics;
    PredPolygonSupport*        polygonSupport;
    ContactDetection*          contactDetector;
    torqueController*          torqController;
    PredictiveModeController*  predModelController;
    PredictiveModeController*  predModelControllerNormal;

    StateEstimation*    stateEstimation;

    FeetTrajectory fl_trajectory;
    FeetTrajectory fr_trajectory;
    FeetTrajectory bl_trajectory;
    FeetTrajectory br_trajectory;

    bool walk;
    bool set_initial_gait_file;
    bool desired_walk;
    float gait_period;
    float gait_period_min_lim;
    float gait_period_max_lim;
    float prop_swing_time;
    float normalize_stance_time;

    float factor_predictive_support_polygon;
    float wfactor_stance_sigma1;
    float wfactor_stance_sigma2;
    float wfactor_swing_sigma1;
    float wfactor_swing_sigma2;
    float wfactor_offset;
    float feet_offset_x;
    float feet_offset_y;
    float base_offset_x;
    float base_offset_y;
    float force_contact_threshold;
    float capture_point_velocity_factor;

    float max_abs_vel_x;
    float max_abs_vel_y;
    float max_abs_vel_z;
    float max_abs_vel_roll;
    float max_abs_vel_pitch;
    float max_abs_vel_yaw;
    float filter_vel_cmd_factor;
    float linear_vel_th_to_walk;
    float linear_ang_vel_th_to_walk;

    float robot_COM_z_pos;

    FilterSmooth filter_vel_cmd;
    FilterSmooth filter_vel_est;


    bool enable_force_controller;
    bool enable_pmc_controller;

    bool available_imu_data;

    long loop_counter;



    Eigen::Vector3f curr_angles_fl, curr_angles_fr, curr_angles_bl, curr_angles_br;
    Eigen::Vector3f prev_angles_fl, prev_angles_fr, prev_angles_bl, prev_angles_br;
    Eigen::Vector3f vel_angles_fl, vel_angles_fr, vel_angles_bl, vel_angles_br;
    bool set_first_ang_vel;

    Eigen::Vector3f curr_feet_fl_pos, curr_feet_fr_pos, curr_feet_bl_pos, curr_feet_br_pos;
    Eigen::Vector3f curr_feet_fl_pos_fromBase, curr_feet_fr_pos_fromBase, curr_feet_bl_pos_fromBase, curr_feet_br_pos_fromBase;
    bool curr_stance_fl, curr_stance_fr, curr_stance_bl, curr_stance_br;

    Eigen::Vector3f estimated_base_pos;
    Eigen::Vector3f locked_base_pos;
    Eigen::Vector3f estimated_base_vel;
    Eigen::Vector3f base_acc_from_base;

    Eigen::Vector4d base_quaternion;
    Eigen::Vector3f base_euler_angles;
    Eigen::Matrix3f base_rotation_matrix;
    Eigen::Vector3f estimated_base_ang_vel;

    Eigen::Vector3f estimated_feet_fl, estimated_feet_fr, estimated_feet_bl, estimated_feet_br;


    Eigen::Vector3f base_position_offset;
    Eigen::Vector3f psp_body_pos;


    Eigen::Vector3f base_vel_weight;
    Eigen::Vector3f des_base_vel, des_base_ang_vel, des_base_pos;
    Eigen::Matrix3f des_base_rot;

    Eigen::Vector3f des_feet_dir_fl, des_feet_dir_fr, des_feet_dir_bl, des_feet_dir_br;

    Eigen::Vector3f des_base_vel_fromBase, estimated_base_vel_fromBase;

    Eigen::Vector3f feet_fl_pos_cmd, feet_fr_pos_cmd, feet_bl_pos_cmd, feet_br_pos_cmd;
    Eigen::Vector3f angles_cmd_fl, angles_cmd_fr, angles_cmd_bl, angles_cmd_br;

    bool motors_disconnected;

    bool contact_fl;
    bool contact_fr;
    bool contact_bl;
    bool contact_br;


};

# endif

