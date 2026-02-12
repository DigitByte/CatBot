#include <ros.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <catbot_msgs/set_face.h>
#include <catbot_msgs/eyes_pos.h>
#include "face.h"


Adafruit_SH1106G displayL = Adafruit_SH1106G(128, 64, &Wire);
Adafruit_SH1106G displayR = Adafruit_SH1106G(128, 64, &Wire1);

Face face(&displayL, &displayR);
float blink_probability = 20;

ros::NodeHandle  nh;

/***************** LOGS *****************/
void logInfo(String msg){
    nh.loginfo(msg.c_str());
}

void logError(String msg){
    nh.logerror(msg.c_str());
}




void setFaceCB(const catbot_msgs::set_face::Request & req, catbot_msgs::set_face::Response & res){
  blink_probability = req.blink_prob * 100;
  if (String(req.style) == String("long_eyes")){
    face.setStyle(LONG_EYES);
  }
  if (String(req.style) == String("square_eyes")){
    face.setStyle(SQUARE_EYES);
  }
  if (String(req.style) == String("wide_eyes")){
    face.setStyle(WIDE_EYES);
  }
  
}


void eyesPosCB(const catbot_msgs::eyes_pos& msg){
  logInfo("called pos");

}



/************************ ROS SERVICES ************************/
ros::ServiceServer<catbot_msgs::set_face::Request, catbot_msgs::set_face::Response> setRobotFaceServer("catbot/set_face",&setFaceCB);


/************************ ROS SUBSCRIBERS ************************/
ros::Subscriber<catbot_msgs::eyes_pos> eyesPosSub("catbot/eyes_pos", &eyesPosCB );


void setup() {
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  
  face.begin();

  delay(10);
  face.drawEyes(0.5,0.5, 0.5, 0.5);
  delay(1000);
  face.blink(0.2, 5);

  
  nh.initNode();

  nh.advertiseService(setRobotFaceServer);
  nh.subscribe(eyesPosSub);
  

}

void loop() {
  float p = random(0, 100);

  if(p < blink_probability){
     face.blink(0.2, 5);
  }


  float p_x = random(25, 75);
  float p_y = random(25, 75);

  p_x = p_x/100;
  p_y = p_y/100;
  face.move(p_x, p_y, 0.1, 5);

  //delay(1);

  nh.spinOnce();

}
