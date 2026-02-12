#include <ros.h>
#include <Wire.h>

#include <sensor_msgs/JointState.h>
#include <catbot_msgs/set_motor_calibration.h>
#include <catbot_msgs/connect_servos.h>
#include <catbot_msgs/contact_detection.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include "SparkFun_BNO080_Arduino_Library.h"



#include "configuration.h"
#include "actuator.h"
#include <Servo.h>
#include "FeetContactDetector.h"







/*********************************************
************* PINS CONFIGURATION *************
**********************************************/


char *servoNames[] = {"hip1_fl", "hip2_fl", "knee_fl",
                      "hip1_fr", "hip2_fr", "knee_fr",
                      "hip1_bl", "hip2_bl", "knee_bl",
                      "hip1_br", "hip2_br", "knee_br"};




int PIN_SERVOS[NUM_MOTORS]  = {2,  1,  0,
                               5,  4,  3,
                               8,  7,  6,
                               11,  10, 9};



int PIN_FEET_CONTACT[4] = {23, 22, 21, 20};



int PIN_SERVO_SWITCH = 14;




/***************** MAIN VARIABLES *****************/
Actuator* servomotors[NUM_MOTORS];
ros::NodeHandle  nh;


                                  
BNO080 imuSensor;  // Check I2C device address and correct line below (by default address is 0x29 or 0x28)id, address

FeetContactDetector stance_fl(PIN_FEET_CONTACT[0]);
FeetContactDetector stance_fr(PIN_FEET_CONTACT[1]);
FeetContactDetector stance_bl(PIN_FEET_CONTACT[2]);
FeetContactDetector stance_br(PIN_FEET_CONTACT[3]);




/***************** LOGS *****************/
void logInfo(String msg){
    nh.loginfo(msg.c_str());
}

void logError(String msg){
    nh.logerror(msg.c_str());
}



/*********************************************
************* CALLBACKS FUNCTIONS ************
**********************************************/


void setMotorCalCB(const catbot_msgs::set_motor_calibration::Request & req, catbot_msgs::set_motor_calibration::Response & res){
  int i = req.servo_index;

  if (!req.setCalibration_servo){
    servomotors[i]->setServoCalibration(false);
  }
  
  else{
    servomotors[i]->setServoCalibration(req.min_calAngle,  req.max_calAngle,
                                        req.min_posServo,  req.max_posServo,
                                        req.min_posAngle,  req.max_posAngle);
                                        
    servomotors[i]->setLimPositions(req.min_lim_angle, req.max_lim_angle);

  }

                                 
  res.result = true;
  logInfo( "Calibration for motor " + servomotors[i]->getName() + " (" + String(i) + "),  Servo cal: " + String( servomotors[i]->isServoCalibrated()));

}



void connectServosCB(const catbot_msgs::connect_servos::Request & req, catbot_msgs::connect_servos::Response & res){
  bool connect = req.connect;
  if (connect){
      for(short i=0; i < NUM_MOTORS; i++){
        servomotors[i]->connect();
      }
      res.result = true;
      logInfo("Servomotors attached");
      digitalWrite(PIN_SERVO_SWITCH, HIGH);
      
  }
  else{
      for(short i=0; i < NUM_MOTORS; i++){
          servomotors[i]->disconnect();
      }
      res.result = true;
      logInfo("Servomotors disattached");
      digitalWrite(PIN_SERVO_SWITCH, LOW);


  }


  
}



void jointControlCB(const sensor_msgs::JointState& msg){


  if(msg.position_length != NUM_MOTORS){
    logError("Invalid joint control msg. Either position array or effort array is empty!.");
    return;
  }

  for(short i=0; i < NUM_MOTORS; i++){
      servomotors[i]->setPosition(msg.position[i]);
  }
}




/************************ ROS SERVICES ************************/

ros::ServiceServer<catbot_msgs::set_motor_calibration::Request, catbot_msgs::set_motor_calibration::Response> setMotorCalServer("catbot/set_motor_calibration",&setMotorCalCB);
ros::ServiceServer<catbot_msgs::connect_servos::Request, catbot_msgs::connect_servos::Response> connectServosServer("catbot/connect_servos",&connectServosCB);


/************************ ROS SUBSCRIBERS ************************/

ros::Subscriber<sensor_msgs::JointState> jointControlSub("catbot/joints_control", &jointControlCB );


/************************ ROS PUBLISHERS ************************/
sensor_msgs::Imu imu_state;
std_msgs::Float64 loop_freq;
catbot_msgs::contact_detection feet_contact_detector;

ros::Publisher IMUStatePub("catbot/imu_data",       &imu_state);
ros::Publisher freqInfoPub("catbot/driver_freq",    &loop_freq);
ros::Publisher contactDetectionPub("catbot/contact_detection",    &feet_contact_detector);



/************************ TIMERS ************************/
unsigned long Timer;
unsigned long TimerPublisher;
unsigned long TimerFreqPubliser;

unsigned int counts_loop;


int imuPublishCounter = 0;


/***************************************************************
 * ********************** SETUP FUNCTION ***********************
 ***************************************************************/


void setup() {


  nh.initNode();
  nh.advertise(IMUStatePub);
  nh.advertise(freqInfoPub);
  nh.advertise(contactDetectionPub);

  nh.advertiseService(setMotorCalServer);
  nh.advertiseService(connectServosServer);

  nh.subscribe(jointControlSub);



  Wire.begin();
  imuSensor.begin();
  Wire.setClock(200000); //Increase I2C data rate to 400kHz
  
  imuSensor.enableGyroIntegratedRotationVector(BNO080_SAMPLERATE_DELAY_MS);
  imuSensor.enableLinearAccelerometer(BNO080_SAMPLERATE_DELAY_MS);  


  
  
  Timer             = micros();
  TimerPublisher    = micros();
  TimerFreqPubliser = micros();


  for(short i=0; i < NUM_MOTORS; i++){
      servomotors[i] = new Actuator(&nh, i, servoNames[i], PIN_SERVOS[i]);
      //servomotors[i]->connect();
      //servomotors[i]->disconnect();
  }

  pinMode(PIN_SERVO_SWITCH, OUTPUT);


    imu_state.orientation.w = 1;
    imu_state.orientation.x = 0;
    imu_state.orientation.y = 0;
    imu_state.orientation.z = 0;

    imu_state.angular_velocity.x =  0;
    imu_state.angular_velocity.y =  0;
    imu_state.angular_velocity.z =  0;


    imu_state.linear_acceleration.x = 0;
    imu_state.linear_acceleration.y = 0;
    imu_state.linear_acceleration.z = 0;

    feet_contact_detector.feet_stance_length = 4;
  
}




void initImuState(){
    imu_state.orientation.w = 1;
    imu_state.orientation.x = 0;
    imu_state.orientation.y = 0;
    imu_state.orientation.z = 0;

    imu_state.angular_velocity.x =  0;
    imu_state.angular_velocity.y =  0;
    imu_state.angular_velocity.z =  0;


    imu_state.linear_acceleration.x = 0;
    imu_state.linear_acceleration.y = 0;
    imu_state.linear_acceleration.z = 0;
}



void getImuData(){
  if (imuSensor.dataAvailable() == false) {
        //logError("Teensy: Cannot connect to imu sensor");
        //return; 
     }

    if (imuPublishCounter == 0){
      initImuState();
    }

    
    float q[4];
    q[0] =   imuSensor.getQuatJ();
    q[1] = - imuSensor.getQuatReal();
    q[2] = - imuSensor.getQuatK();
    q[3] = - imuSensor.getQuatI();
  
    imu_state.orientation.w = -q[0];
    imu_state.orientation.x =  q[1];
    imu_state.orientation.y =  q[2];
    imu_state.orientation.z = -q[3];

    imu_state.angular_velocity.x +=  -imuSensor.getFastGyroY();
    imu_state.angular_velocity.y +=  -imuSensor.getFastGyroX();
    imu_state.angular_velocity.z +=  -imuSensor.getFastGyroZ();


    imu_state.linear_acceleration.x +=  -imuSensor.getLinAccelY();
    imu_state.linear_acceleration.y +=  -imuSensor.getLinAccelX();
    imu_state.linear_acceleration.z +=  -imuSensor.getLinAccelZ();

    
    imuPublishCounter += 1;
}














/***************************************************************
 * **************** PUBLISH DATA FUNCTION ***************
 ***************************************************************/

void sendRobotData(){
    if(micros() - TimerPublisher < 1000000/PUBLISH_DATA_FREQ){
      return;
    }

    

    /************ IMU STATE ***********/

    if(imuPublishCounter > 0){
      imu_state.angular_velocity.x = imu_state.angular_velocity.x/imuPublishCounter;
      imu_state.angular_velocity.y = imu_state.angular_velocity.y/imuPublishCounter;
      imu_state.angular_velocity.z = imu_state.angular_velocity.z/imuPublishCounter;
  
  
      imu_state.linear_acceleration.x = imu_state.linear_acceleration.x/imuPublishCounter;
      imu_state.linear_acceleration.y = imu_state.linear_acceleration.y/imuPublishCounter;
      imu_state.linear_acceleration.z = imu_state.linear_acceleration.z/imuPublishCounter;
    }

    
    imu_state.header.stamp = nh.now();
    imu_state.header.frame_id = "base_link";
    IMUStatePub.publish( &imu_state );
    

    imuPublishCounter = 0;


    imu_state.angular_velocity.x =  0;
    imu_state.angular_velocity.y =  0;
    imu_state.angular_velocity.z =  0;


    imu_state.linear_acceleration.x = 0;
    imu_state.linear_acceleration.y = 0;
    imu_state.linear_acceleration.z = 0;

    TimerPublisher = micros();    

    feet_contact_detector.header.stamp = nh.now();

    //feet_contact_detector.feet_stance[0] = 0;
    //feet_contact_detector.feet_stance[1] = 0;
    //feet_contact_detector.feet_stance[2] = 0;
    //feet_contact_detector.feet_stance[3] = 0;

    bool stance[4] = {stance_fl.getStance(), stance_fr.getStance(), stance_bl.getStance(), stance_br.getStance()};

    //int val[4] = {stance_fl.get_analog_val(), stance_fr.get_analog_val(), stance_bl.get_analog_val(), stance_br.get_analog_val()};
    //logInfo(String(val[0]));
    
    feet_contact_detector.feet_stance = stance;
    contactDetectionPub.publish(&feet_contact_detector);
}








/***************************************************************
 * **************** PUBLISH FREQ FUNCTION **********************
 ***************************************************************/

 
void publishInfoFreq(){
    unsigned long deltaT = micros()  - TimerFreqPubliser;
    if(deltaT < 1000000/PUBLISH_INFO_FREQ){
      return;
    }

    double freq = double(deltaT)/double(counts_loop);
    freq = 1000000/freq;

    loop_freq.data = freq;
    freqInfoPub.publish( &loop_freq );


    TimerFreqPubliser = micros();
    counts_loop = 0;
}








/***************************************************************
 * **************** LOOP FUNCTION *****************************
 ***************************************************************/

void loop(){
  double dt = float(micros() - Timer) / 1000000.0;
  Timer = micros();
  for(short i=0; i < NUM_MOTORS; i++){
      servomotors[i]->run(dt);
  }
  getImuData();


  sendRobotData();
  nh.spinOnce();

  publishInfoFreq();
  counts_loop += 1;
}
