

/*
 * Arduino/ROS Interface Program
 * Casimir Sowinski, Alex Renaud, 2016
 */

// Check Arduino version
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// Includes
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
void teleop_cb(const geometry_msgs::Twist&);
void trans_cb(const std_msgs::Float64MultiArray&);
void joint_state_cb(const std_msgs::Float64MultiArray&);
void position_cb(const std_msgs::Float64&);

void head_cb(const std_msgs::Float64MultiArray&);
void right_arm_cb(const std_msgs::Float64MultiArray&);
void right_gripper_cb(const std_msgs::Float64MultiArray&);
void left_arm_cb(const std_msgs::Float64MultiArray&);
void left_gripper_cb(const std_msgs::Float64MultiArray&);

void setup_USE_PIN();

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
/*
torso_head_tele_joint:     {id:  1, neutral: 512},
head_pan_joint:            {id:  2, neutral: 512, min_angle:  -90, max_angle: 90},
head_tilt_joint:           {id:  3, neutral: 512, min_angle:  -45, max_angle: 108},        
**  
right_arm_out_joint:       {id:  4, neutral: 512, min_angle:    0, max_angle: 90},
right_arm_fwd_joint:       {id:  5, neutral: 512, min_angle:  -90, max_angle: 90},
right_arm_rotate_joint:    {id:  6, neutral: 512, min_angle:  -90, max_angle: 90},
right_elbow_joint:         {id:  7, neutral: 512, min_angle:    0, max_angle: 90},
right_wrist_joint:         {id:  8, neutral: 512, min_angle:    0, max_angle: 180},    
** 
right_gripper_index_joint: {id:  9, neutral: 512},
right_gripper_thumb_joint: {id: 10, neutral: 512},    
** 
left_arm_out_joint:        {id: 11, neutral: 512, min_angle:    0, max_angle: 90},
left_arm_fwd_joint:        {id: 12, neutral: 512, min_angle:  -90, max_angle: 90},
left_arm_rotate_joint:     {id: 13, neutral: 512, min_angle:  -90, max_angle: 90},
left_elbow_joint:          {id: 14, neutral: 512, min_angle:    0, max_angle: 90},
left_wrist_joint:          {id: 15, neutral: 512, min_angle:    0, max_angle: 180},    
** 
left_gripper_index_joint:  {id: 16, neutral: 512},
left_gripper_thumb_joint:  {id: 17, neutral: 512}    
*/

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


// Replace this with a common parameter b/t the translate node and this that
// looks at whether the limb is enabled globally
const int USE_HEAD = 1;
const int USE_RIGHT_ARM = 1;
const int USE_RIGHT_GRIPPER = 0;
const int USE_LEFT_ARM = 0;
const int USE_LEFT_GRIPPER = 0;
// Whether to use this Pololu pin (not actually a const)
int USE_PIN[18] = {0};
// Keeps initial angles for servos
const int INIT_ANGLE[18] = {90}; 
// Sizes of limb position arrays
const int HEAD_SIZE = 3;
const int RIGHT_ARM_SIZE = 5;
const int RIGHT_GRIPPER_SIZE = 2;
const int LEFT_ARM_SIZE = 5;
const int LEFT_GRIPPER_SIZE = 2;
// Starting Pololu pins of limbs
const int HEAD_SP = 1;
const int RIGHT_ARM_SP = 4;
const int RIGHT_GRIPPER_SP = 9;
const int LEFT_ARM_SP = 11;
const int LEFT_GRIPPER_SP = 16;

const float SCALE = 4.0;

const float pi = 3.14159;
const int pan_lim_h = 180;
const int pan_lim_l = 0;
const int til_lim_h = 165;
const int til_lim_l = -90;
int pubFlag = 1; // Used to run arbotix publish once

// Nodes, pubs, and subs
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_teleop("turtle1/cmd_vel", teleop_cb);
ros::Subscriber<std_msgs::Float64MultiArray> sub_trans("arduino_joint", trans_cb);

ros::Subscriber<std_msgs::Float64MultiArray> head_sub("angles/head", head_cb);
ros::Subscriber<std_msgs::Float64MultiArray> right_arm_sub("angles/right_arm", right_arm_cb);
ros::Subscriber<std_msgs::Float64MultiArray> right_gripper_sub("angles/right_gripper", right_gripper_cb);
ros::Subscriber<std_msgs::Float64MultiArray> left_arm_sub("angles/left_arm", left_arm_cb);
ros::Subscriber<std_msgs::Float64MultiArray> left_gripper_sub("angles/left_gripper", left_gripper_cb);

ros::Publisher pan("head_pan_joint/command", &pan_ang_rad);
ros::Publisher til("head_tilt_joint/command", &til_ang_rad);

//-----------------Setup
void setup(){
  // Populate USE_SP[] based on whether limbs are enabled
  setup_USE_PIN();
  
  // Assign vars  
  pan_ang.data = 90.0;
  til_ang.data = 90.0;
  pan_ang_old.data = 90.0;
  til_ang_old.data = 90.0;  
  servo_test.data = 90.0;
    
  // Setup servos
  maestroSerial.begin(9600);  
  for(int i = 0; i < 18; i++){
    if(USE_PIN[i]){
      maestro.setSpeed(i, 10);
      maestro.setAcceleration(i, 127);
      maestro.setTarget(i, deg2Polo(90));
    }
  }

  // Setup ROS node
  nh.initNode();
  
  // Handle pubs and subs
  nh.subscribe(sub_teleop);
  nh.subscribe(sub_trans);
  
  if(USE_HEAD){
    nh.subscribe(head_sub);
  }
  if(USE_RIGHT_ARM){
    nh.subscribe(right_arm_sub);
  }
  if(USE_RIGHT_GRIPPER){
    nh.subscribe(right_gripper_sub);
  }
  if(USE_LEFT_ARM){
    nh.subscribe(left_arm_sub);
  }
  if(USE_LEFT_GRIPPER){
    nh.subscribe(left_gripper_sub);
  }
  
  //nh.subscribe(right_arm_sub);
  nh.advertise(pan);
  nh.advertise(til); 

}

//-----------------Main
void loop(){    
  /*
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
  */  
  //maestro.setTarget(servo_pin_pan, rad2Polo(servo_test.data));  
  // Process callbacks and wait
  nh.spinOnce();
  delay(1);
}

//-----------------Function Definitions

void head_cb(const std_msgs::Float64MultiArray& j_state){
  //nh.loginfo("Entered head_cb");   
  for (int i = 0; i < HEAD_SIZE; i++){
    maestro.setTarget(i + HEAD_SP, rad2Polo(SCALE * j_state.data[i]));
  }
}

void right_arm_cb(const std_msgs::Float64MultiArray& j_state){
  //nh.loginfo("Entered right_arm_cb"); 
  for (int i = 0; i < RIGHT_ARM_SIZE; i++){
    maestro.setTarget(i + RIGHT_ARM_SP, rad2Polo(SCALE * 4 * j_state.data[i] + PI / 2));
  }
}

void right_gripper_cb(const std_msgs::Float64MultiArray& j_state){
  //nh.loginfo("Entered right_gripper_cb"); 
  for (int i = 0; i < RIGHT_GRIPPER_SIZE; i++){
    maestro.setTarget(i + RIGHT_GRIPPER_SP, rad2Polo(j_state.data[i]));
  }
}

void left_arm_cb(const std_msgs::Float64MultiArray& j_state){
  //nh.loginfo("Entered left_arm_cb"); 
  for (int i = 0; i < LEFT_ARM_SIZE; i++){
    maestro.setTarget(i + LEFT_ARM_SP, rad2Polo(j_state.data[i]));
  }
}

void left_gripper_cb(const std_msgs::Float64MultiArray& j_state){
  //nh.loginfo("Entered left_gripper_cb"); 
  for (int i = 0; i < LEFT_GRIPPER_SIZE; i++){
    maestro.setTarget(i + LEFT_GRIPPER_SP, rad2Polo(j_state.data[i]));
  }
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
  return 4000 + 4000 * ang_rad / PI / 2;
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

// Goes through USE_PIN[] and sets the elements to 1 if the related limb is enabled
void setup_USE_PIN(){
  int offset = 0;  
  for(int i = offset; i < offset + HEAD_SIZE; i++){
    if(USE_HEAD){
      USE_PIN[i] = 1;
    }    
  }  
  offset += HEAD_SIZE;  
  for(int i = offset; i < offset + RIGHT_ARM_SIZE; i++){
    if(USE_RIGHT_ARM){
      USE_PIN[i] = 1;
    }    
  }
  offset += RIGHT_ARM_SIZE;  
  for(int i = offset; i < offset + RIGHT_GRIPPER_SIZE; i++){
    if(USE_RIGHT_GRIPPER){
      USE_PIN[i] = 1;
    }    
  }
  offset += RIGHT_GRIPPER_SIZE;  
  for(int i = offset; i < offset + LEFT_ARM_SIZE; i++){
    if(USE_LEFT_ARM){
      USE_PIN[i] = 1;
    }    
  }
  offset += LEFT_ARM_SIZE;  
  for(int i = offset; i < offset + LEFT_GRIPPER_SIZE; i++){
    if(USE_LEFT_GRIPPER){
      USE_PIN[i] = 1;
    }    
  }  
}
