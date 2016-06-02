/*
 * Casimir Sowinski, 2016
 * This node subscribes to /joint_states (sensor_msgs::JointState) and publishes 
 * a Float64MultiArray message for the Arduino node over rosserial
 *
 */

//----------Dependencies
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

//----------Function prototypes
void joint_cb(const sensor_msgs::JointState::ConstPtr&);
void initArrays();
void publishArrays();

//----------Global vars
// Arrays that hold joint angles for limbs 
std_msgs::Float64MultiArray headArray;
std_msgs::Float64MultiArray rightArmArray;
std_msgs::Float64MultiArray rightGripperArray;
std_msgs::Float64MultiArray leftArmArray;
std_msgs::Float64MultiArray leftGripperArray;
// Sizes of position arrays
const int HEAD_SIZE           = 3;
const int RIGHT_ARM_SIZE      = 5;
const int RIGHT_GRIPPER_SIZE  = 2;
const int LEFT_ARM_SIZE       = 5;
const int LEFT_GRIPPER_SIZE   = 2;
// Enable(1) or disable(0) limb publishing
const bool USE_HEAD           = 1;
const bool USE_RIGHT_ARM      = 1;
const bool USE_RIGHT_GRIPPER  = 1;
const bool USE_LEFT_ARM       = 0;
const bool USE_LEFT_GRIPPER   = 0;

//----------Main
int main(int argc, char **argv){
  
  // Pass argc and argv to ros::init() so it can remap CL arguments
  ros::init(argc, argv, "translate");
  
  // Init node so ROS can interact with it
  ros::NodeHandle nh;

  // Print message
  nh.loginfo("Initializing translate node");

  // Subscribe to joint_states to pull all angles, velocities, accelerations, etc.
  ros::Subscriber arduino_joint_sub = nh.subscribe("joint_states", 100, joint_cb);

  // Publish
  ros::Publisher head_pub           = nh.advertise<std_msgs::Float64MultiArray>("angles/head", 100);
  ros::Publisher right_arm_pub      = nh.advertise<std_msgs::Float64MultiArray>("angles/right_arm", 100);
  ros::Publisher right_gripper_pub  = nh.advertise<std_msgs::Float64MultiArray>("angles/right_gripper", 100);
  ros::Publisher left_arm_pub       = nh.advertise<std_msgs::Float64MultiArray>("angles/left_arm", 100);
  ros::Publisher left_gripper_pub   = nh.advertise<std_msgs::Float64MultiArray>("angles/left_gripper", 100);

  // Init vars
  ros::Rate loop_rate(20);        // Makes the while loop spin at 20 Hz
  initArrays();    

  // Print message
  nh.loginfo("Set up publisher(s) and subscriber");
  if(USE_HEAD){
    nh.loginfo("Using Head");
  }
  if(USE_RIGHT_ARM){
    nh.loginfo("Using Right Arm");
  }
  if(USE_RIGHT_GRIPPER){
    nh.loginfo("Using Right Gripper");
  }
  if(USE_LEFT_ARM){
    nh.loginfo("Using Left Arm");
  }
  if(USE_LEFT_GRIPPER){
    nh.loginfo("Using Left Gripper");
  }     

  // Loop and handle SIGINT (etc.) interruption
  while (ros::ok()){    
    // Publish processed array(s)
    //publishArrays();
    if(USE_HEAD){
    head_pub.publish(headArray);  
    }
    if(USE_RIGHT_ARM){
      right_arm_pub.publish(rightArmArray);  
    }
    if(USE_RIGHT_GRIPPER){
      right_gripper_pub.publish(rightGripperArray);  
    }
    if(USE_LEFT_ARM){
      left_arm_pub.publish(leftArmArray);  
    }
    if(USE_LEFT_GRIPPER){
      left_gripper_pub.publish(leftGripperArray);  
    }   
    // spin/wait
    ros::spinOnce();
    loop_rate.sleep();
    //ROS_INFO("console test message");
  } 
  return 0;
}

//----------Function definitions
// Go through joint_state/position and divy up angles to seperate arrays to send over rosserial if they are enabled 
void joint_cb(const sensor_msgs::JointState::ConstPtr& joint_msg){
  // Print message
  //ROS_INFO("entered joint_cb"); 
  // Keeps track of offset in joint_msg array 
  int offset = 0; 
  // Copy position data to appropriate arrays
  if(USE_HEAD){
    for (int i = 0; i < HEAD_SIZE; i++){
      headArray.data[i] = joint_msg->position[i + offset];
    }  
  }    
  offset = HEAD_SIZE;
  if(USE_RIGHT_ARM){
    for (int i = 0; i < RIGHT_ARM_SIZE; i++){
      rightArmArray.data[i] = joint_msg->position[i + offset];
    }  
  }
  offset += RIGHT_ARM_SIZE;
  if(USE_RIGHT_GRIPPER){
    for (int i = 0; i < RIGHT_GRIPPER_SIZE; i++){
      rightGripperArray.data[i] = joint_msg->position[i + offset];
    }  
  }
  offset += RIGHT_GRIPPER_SIZE;
  if(USE_LEFT_ARM){
    for (int i = 0; i < LEFT_ARM_SIZE; i++){
      leftArmArray.data[i] = joint_msg->position[i + offset];
    }  
  }
  offset += LEFT_ARM_SIZE;
  if(USE_LEFT_GRIPPER){
    for (int i = 0; i < LEFT_GRIPPER_SIZE; i++){
      leftGripperArray.data[i] = joint_msg->position[i + offset];
    }  
  }  
}
// Set array size and initialize to 0's
void initArrays(){
  // Clear/init arrays  
  headArray.data.clear();
  rightArmArray.data.clear();
  rightGripperArray.data.clear();
  leftArmArray.data.clear();
  leftGripperArray.data.clear();
  // Set array sizes, change this later, maybe  
  for (int i = 0; i < HEAD_SIZE; i++)
  {
    headArray.data.push_back(0);
  }
  for (int i = 0; i < RIGHT_ARM_SIZE; i++)
  {
    rightArmArray.data.push_back(0);
  }
  for (int i = 0; i < RIGHT_GRIPPER_SIZE; i++)
  {
    rightGripperArray.data.push_back(0);
  }
  for (int i = 0; i < LEFT_ARM_SIZE; i++)
  {
    leftArmArray.data.push_back(0);
  }
  for (int i = 0; i < LEFT_GRIPPER_SIZE; i++)
  {
    leftGripperArray.data.push_back(0);
  }
}
// Publish enabled arrays
void publishArrays(){
/*
  if(USE_HEAD){
    head_pub.publish(headArray);  
  }
  if(USE_RIGHT_ARM){
    right_arm_pub.publish(rightArmArray);  
  }
  if(USE_RIGHT_GRIPPER){
    right_gripper_pub.publish(rightGripperArray);  
  }
  if(USE_LEFT_ARM){
    left_arm_pub.publish(leftArmArray);  
  }
  if(USE_LEFT_GRIPPER){
    left_gripper_pub.publish(leftGripperArray);  
  }  
  */
}




