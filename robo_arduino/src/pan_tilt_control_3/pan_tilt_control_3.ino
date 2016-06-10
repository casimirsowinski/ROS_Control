/************************************OLD****************************************
Package: robo_arduino
Version: 0.0.3
Description: This program subscribes to tele_op and controls a pan/tilt camera 
gimbal and updates joint_states/RViz. 
Maintainer: Casimir Sowinski, "casimirsowinski@gmail.com"
License: BSD
Repo: https://github.com/casimirsowinski/robo_hand_01.git
Author: Casimir Sowinski, "casimirsowinski@gmail.com"
Year: 2016
**************************************OLD**************************************/

// Includes
// Check Arduino version, include the appropriate file
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <string.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <PololuMaestro.h>

// Prototypes 
void servo_cb(const sensor_msgs::JointState&);
void test_cb(const std_msgs::UInt16&);
void teleop_cb(const geometry_msgs::Twist&);
void trans_cb(const std_msgs::Float64MultiArray&);
void joint_state_cb(const std_msgs::Float64MultiArray&);
void position_cb(const std_msgs::Float64&);
float rad2Polo(float);
float deg2Polo(float);
float deg2rad(float);
void pubPanRad();
void pubTilRad();

// Init ROS vars
std_msgs::String str_msg;
sensor_msgs::JointState state_g;
std_msgs::Float64 pan_ang_del;
std_msgs::Float64 til_ang_del;
std_msgs::Float64 pan_ang;
std_msgs::Float64 til_ang;
std_msgs::Float64 pan_ang_old;
std_msgs::Float64 til_ang_old;
std_msgs::Float64 pan_ang_rad;
std_msgs::Float64 til_ang_rad;

std_msgs::Float64 servo_test;
std_msgs::Float64 position_test;

std_msgs::Float64MultiArray j_state;

// Init Pololu vars
#define maestroSerial Serial1
MiniMaestro maestro(Serial1);

// Init Arduino vars
const int servo_pin_pan = 0;
const int servo_pin_til = 1;

const int servo_pin_right_arm_out = 2;
const int servo_pin_right_arm_forward = 3;
const int servo_pin_right_arm_rotate = 4;
const int servo_pin_right_elbow_joint = 5;
const int servo_pin_right_wrist = 6;
const int servo_pin_right_gripper_index = 7;
const int servo_pin_right_gripper_thumb = 8;



char hello[13] = "hello world!";
const float pi = 3.14159;
const int pan_lim_h = 180;
const int pan_lim_l = 0;
const int til_lim_h = 165;
const int til_lim_l = -90;
int pubFlag = 1; // Used to run arbotix publish once

// Nodes, pubs, and subs
ros::NodeHandle nh;
//ros::Subscriber<sensor_msgs::JointState> sub_joint2("joint_states", servo_cb);
//ros::Subscriber<std_msgs::Float64MultiArray> sub_joint("joint_states/position", joint_state_cb);
//ros::Subscriber<std_msgs::Float64> sub_pos("joint_states/position[3]", position_cb);

ros::Subscriber<std_msgs::Float64MultiArray> sub_trans("arduino_joint", trans_cb);
ros::Subscriber<geometry_msgs::Twist> sub_teleop("turtle1/cmd_vel", teleop_cb);
ros::Publisher pan("head_pan_joint/command", &pan_ang_rad);
ros::Publisher til("head_tilt_joint/command", &til_ang_rad);

// Function definitions
void setup(){
  // Assign vars  
  pan_ang.data = 90.0;
  til_ang.data = 90.0;
  pan_ang_old.data = 90.0;
  til_ang_old.data = 90.0;
  
  servo_test.data = 90.0;
    
  // Setup servos
  maestroSerial.begin(9600);
  
  maestro.setSpeed(servo_pin_pan, 10);
  maestro.setAcceleration(servo_pin_pan, 127);
  maestro.setSpeed(servo_pin_til, 10);
  maestro.setAcceleration(servo_pin_til, 127);
  maestro.setSpeed(servo_pin_right_arm_out, 10);
  maestro.setAcceleration(servo_pin_right_arm_out, 127);
  
  // Init servos
  maestro.setTarget(servo_pin_pan, deg2Polo(pan_ang.data));
  maestro.setTarget(servo_pin_til, deg2Polo(til_ang.data));
  maestro.setTarget(servo_pin_right_arm_out, deg2Polo(90));  
  maestro.setTarget(servo_pin_right_arm_forward, deg2Polo(90));
  maestro.setTarget(servo_pin_right_arm_rotate, deg2Polo(90));
  maestro.setTarget(servo_pin_right_elbow_joint, deg2Polo(90));
  maestro.setTarget(servo_pin_right_wrist, deg2Polo(90));
  maestro.setTarget(servo_pin_right_gripper_index, deg2Polo(90));
  maestro.setTarget(servo_pin_right_gripper_thumb, deg2Polo(90));
   
  // Setup ROS node
  nh.initNode();
  
  // Handle pubs and subs
  nh.subscribe(sub_teleop);
  nh.subscribe(sub_trans);
  nh.advertise(pan);
  nh.advertise(til); 
  //nh.subscribe(sub_joint2);
  //nh.subscribe(sub_pos);
  //nh.subscribe(sub_joint);
  //nh.subscribe(sub_test);  

}

void loop(){    
  // Initial publish
  if (pubFlag == 1) {      
    pubPanRad();
    pubTilRad();
    pubFlag = 0;
    nh.loginfo("Entered pub thingie"); 
  }
      
  // Publish if angles have been updated
  if (pan_ang_old.data != pan_ang.data) {
    pubPanRad();
    pan_ang_old.data = pan_ang.data;
  }
  if (til_ang_old.data != til_ang.data) {
    pubTilRad();
    til_ang_old.data = til_ang.data;
  }     
    
  //maestro.setTarget(servo_pin_pan, rad2Polo(servo_test.data));  
  // Process callbacks and wait
  nh.spinOnce();
  delay(1);
}

void trans_cb(const std_msgs::Float64MultiArray& j_state){
  delay(3000);
  nh.loginfo("Entered trans_cb"); 
  delay(3000);
  for (int i = 0; i <=6; i++){
  maestro.setTarget(i+2, rad2Polo(j_state.data[i]));
  }
}

void teleop_cb(const geometry_msgs::Twist& cmd){      
  pan_ang_del.data = cmd.angular.z / 2;
  til_ang_del.data = cmd.linear.x / 2;  
  pan_ang.data += pan_ang_del.data; 
  til_ang.data += til_ang_del.data;
  
  // Limit checking
  if (pan_ang.data > pan_lim_h) 
  { pan_ang.data = pan_lim_h; }
  if (pan_ang.data < pan_lim_l) 
  { pan_ang.data = pan_lim_l; } 
  if (til_ang.data > til_lim_h) 
  { til_ang.data = til_lim_h; }
  if (til_ang.data < til_lim_l) 
  { til_ang.data = til_lim_l; }
  
  // Send commands to servos 
  maestro.setTarget(servo_pin_pan, deg2Polo(pan_ang.data));
  maestro.setTarget(servo_pin_til, deg2Polo(til_ang.data));
  
  nh.loginfo("Entered cb");
}

// Convert radians to Pololu
float rad2Polo(float ang_rad){
  return 4000 + 4000 * ang_rad / 3.14159;
}

// Convert degrees to Pololu (0-180->4000-8000)
float deg2Polo(float ang_deg){
  return 4000 + 4000 * ang_deg / 180;   
}

//Convert degrees to radians
float deg2rad(float ang_deg){
  return ang_deg * 3.14159 / 180;
}

// Convert degrees to radians, offsets, and publishes to Arbotix
// magic number '42' accounts for physical setup
void pubPanRad(){
  pan_ang_rad.data = deg2rad(42 - pan_ang.data / 2);
  pan.publish(&pan_ang_rad);
}

// magic number '5' accounts for physical setup
void pubTilRad(){
  til_ang_rad.data = deg2rad(til_ang.data / 2 + 5);
  til.publish(&til_ang_rad);
}

void servo_cb(const sensor_msgs::JointState& state){  
  //nh.loginfo("entered j2"); 
  //j_state2.data = state.position;
  //state_g.position = {0.0 1.1 2.2 3.3};  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void position_cb(const std_msgs::Float64& pos){
  
  nh.loginfo("Entered position_cb"); 
  
}

