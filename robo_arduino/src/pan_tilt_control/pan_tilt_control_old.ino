/************************************OLD****************************************
Package: robo_arduino
Version: 0.0.1
Description: This program subscribes to tele_op and controls a pan/tilt camera 
gimbal and updates joint_states/RViz.
controller. 
Maintainer: Casimir Sowinski, "casimirsowinski@gmail.com"
License: BSD
Repo: https://github.com/casimirsowinski/robo_hand_01.git
Author: Casimir Sowinski, "casimirsowinski@gmail.com"
Year: 2016
**************************************OLD**************************************/

// Includes
// Check Arduino version, include the appropriate file
// Check Arduino version
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// Includes
#include <Servo.h> 
#include <ros.h>
#include <string.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

// Prototypes 
void servo_cb(const std_msgs::Float64MultiArray&);
void test_cb(const std_msgs::UInt16&);
void teleop_cb(const geometry_msgs::Twist&);

// Init ROS vars
std_msgs::String str_msg;

sensor_msgs::JointState state_g;
sensor_msgs::JointState jointTest;
sensor_msgs::JointState jointTest_old;
std_msgs::Float64MultiArray posArray;
std_msgs::Float64 pos;

std_msgs::Float64 pan_ang_del;
std_msgs::Float64 tilt_ang_del;
std_msgs::Float64 pan_ang;
std_msgs::Float64 tilt_ang;
std_msgs::Float64 pan_ang_old;
std_msgs::Float64 tilt_ang_old;
std_msgs::Float64 pan_ang_rad;
std_msgs::Float64 tilt_ang_rad;

// Init Arduino vars
Servo servo_pan;
Servo servo_tilt;
char hello[13] = "hello world!";
int n = 0;
int up_dn = 1;
String buf_str;
float pi = 3.14159;

// Nodes, pubs, and subs
ros::NodeHandle nh;

//ros::Subscriber<std_msgs::UInt16> sub_test("test_msg", test_cb);
//ros::Subscriber<sensor_msgs::JointState> sub_joint("joint_states", servo_cb);
ros::Subscriber<std_msgs::Float64MultiArray> sub_joint("joint_states/position", servo_cb);
ros::Subscriber<geometry_msgs::Twist> sub_teleop("turtle1/cmd_vel", teleop_cb);

ros::Publisher pan("head_pan_joint/command", &pan_ang_rad);
ros::Publisher tilt("head_tilt_joint/command", &tilt_ang_rad);

ros::Publisher pub_joint("joint_test", &pos);
//ros::Publisher chatter("chatter", &str_msg);
//ros::Publisher state("state", &state_g);

// Function definitions
void setup(){
  // Assign vars  int 
  pan_ang.data = 90.0;
  tilt_ang.data = 90.0;
  pan_ang_old.data = 90.0;
  tilt_ang_old.data = 90.0;
   
  pos.data = 5.2;
  
  // Setup servos
  servo_pan.attach(10); // attach pan servo to pin 9
  servo_tilt.attach(9); // attach tilt servo to pin 10
  pinMode(13, OUTPUT);

  // Setup ROS node
  nh.initNode();
  
  // Handle pubs and subs
  nh.subscribe(sub_joint);
  //nh.subscribe(sub_test);
  //nh.subscribe(sub_teleop);
  nh.advertise(pan);
  nh.advertise(tilt);
  nh.advertise(pub_joint);
  //nh.advertise(chatter);
  //nh.advertise(state);
  
}

void teleop_cb(const geometry_msgs::Twist& cmd){    
  //geometry_msgs::Twist cmd_lim = cmd;  
  //std_msgs::Float64 pan_ang_del;
  //std_msgs::Float64 tilt_ang_del;
  
  pan_ang_del.data = cmd.angular.z / 2;
  tilt_ang_del.data = cmd.linear.x / 2;
  
  pan_ang.data += pan_ang_del.data; 
  tilt_ang.data += tilt_ang_del.data;
  
  if (pan_ang.data > 180) 
  { pan_ang.data = 180; }
  if (pan_ang.data < 0) 
  { pan_ang.data = 0; }  
  
  if (tilt_ang.data > 165) 
  { tilt_ang.data = 165; }
  if (tilt_ang.data < 0) 
  { tilt_ang.data = 0; }
  
  servo_pan.write(pan_ang.data); //set servo angle, should be from 0-180  
  servo_tilt.write(tilt_ang.data); //set servo angle, should be from 0-180    
}

void servo_cb(const std_msgs::Float64MultiArray& positionArray){  
  
  pos.data = positionArray.data[4];
  
  pos.data = 3.3;
  
  //state_g = state; 
  //state_g.position = {0.0 1.1 2.2 3.3};  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void test_cb(const std_msgs::UInt16& cmd_msg){
  if(up_dn) {
    if (n < 90) {
      n++;
    } else {
      up_dn = 0;
    }
  } else {
    if (n > 0) {
      n--;
    } else {
      up_dn = 1;
    }
  }
  
  //servo.write(n); //set servo angle, should be from 0-180  
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void loop(){  
  
  // Publish
  if (pan_ang_old.data != pan_ang.data) {
    pan_ang_rad.data = pan_ang.data * pi / 180 - pi / 2;
    pan.publish(&pan_ang_rad);
    pan_ang_old.data = pan_ang.data;
  }
  if (tilt_ang_old.data != tilt_ang.data) {
    tilt_ang_rad.data = tilt_ang.data * pi / 180 - pi / 4;
    tilt.publish(&tilt_ang_rad);
    tilt_ang_old.data = tilt_ang.data;
  }      
  
  pub_joint.publish(&pos);
  
  // Process callbacks and wait
  nh.spinOnce();
  //nh.spin();
  delay(1);
}










