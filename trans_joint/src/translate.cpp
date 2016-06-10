/********************************************************************************
Package: trans_joint
Version: 0.0.1
Description: This node subscribes to /joint_states (sensor_msgs::JointState) and 
publishes multiple Float64MultiArray messages for the Arduino node over rosserial
Maintainer: Casimir Sowinski, "casimirsowinski@gmail.com"
License: BSD
Repo: https://github.com/casimirsowinski/robo_hand_01.git
Author: Casimir Sowinski, "casimirsowinski@gmail.com"
Year: 2016
*******************************************************************************/

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
void printHeader();

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
// Arrays that hold the index for joints in joint_states
int head_idx[HEAD_SIZE];
int right_arm_idx[RIGHT_ARM_SIZE];
int right_gripper_idx[RIGHT_GRIPPER_SIZE];
int left_arm_idx[LEFT_ARM_SIZE];
int left_gripper_idx[LEFT_GRIPPER_SIZE];
// Flag to track whether to form index arrays
int firstFlag = 1;

//----------Main
int main(int argc, char **argv){
  
  // Pass argc and argv to ros::init() so it can remap CL arguments
  ros::init(argc, argv, "translate");
  
  // Init node so ROS can interact with it
  ros::NodeHandle nh;

  // Print message
  printHeader();

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
  
  if(firstFlag){
    ROS_INFO("Initializing index arrays...");
    //int num_elements = sizeof(joint_msg->name)/sizeof(joint_msg->name[0]);
    int num_elements = 17; //lol
    
    ROS_INFO("num_elements: %d", num_elements);
//    const std::basic_string<char> temp = joint_msg->name[0];
    
    //std::string temp = joint_msg->name[0];
    
    
    
    //s.data = c.c_str(); //same data as above.
    //pthread_mutex_lock(&send_CS);
    //tum_ardrone_pub.publish(s);
    //pthread_mutex_unlock(&send_CS);
    
    
    
    //const std::stringstream poop;
    
    //poop = temp;
    
    std_msgs::String s;
    /*
    s.data = joint_msg->name[0];        
    char * cstr = new char [s.data.length()+1];
    std::strcpy (cstr, s.data.c_str());
    
    if(s.data.compare("head_pan_joint") == 0){
      ROS_INFO("success");
    }
    
    ROS_INFO("cstr: %s", cstr);  
    ROS_INFO("ROS string: %s", s.data.c_str());
    */
    
    // loop through names, compare them with literals in order and save index of where they are
    // in joint_states/name array
    for(int i = 0; i < num_elements; i++){    
      // Get name at index i
      s.data = joint_msg->name[i];      
      
      ROS_INFO("i: %d, name: %s", i, s.data.c_str());
              
      // check if there is a match with the element in name array, save the index in the 
      // appropriate index array at the correct position
      // head
      if(s.data.compare("torso_head_tele_joint") == 0){
        head_idx[0] = i;
        ROS_INFO("tele index: %d", i);
      }
      else if(s.data.compare("head_pan_joint") == 0){
        head_idx[1] = i;
        ROS_INFO("pan index: %d", i);
      }
      else if(s.data.compare("head_tilt_joint") == 0){
        head_idx[2] = i;
        ROS_INFO("tilt index: %d", i);
      }
      // right arm
      else if(s.data.compare("right_arm_out_joint") == 0){
        right_arm_idx[0] = i;
        ROS_INFO("rao index: %d", i);
      }
      else if(s.data.compare("right_arm_fwd_joint") == 0){
        right_arm_idx[1] = i;
        ROS_INFO("raf index: %d", i);
      }
      else if(s.data.compare("right_arm_rotate_joint") == 0){
        right_arm_idx[2] = i;
        ROS_INFO("rar index: %d", i);
      }
      else if(s.data.compare("right_elbow_joint") == 0){
        right_arm_idx[3] = i;
        ROS_INFO("re index: %d", i);
      }
      else if(s.data.compare("right_wrist_joint") == 0){
        right_arm_idx[4] = i;
        ROS_INFO("rw index: %d", i);
      }
      // right gripper
      else if(s.data.compare("right_gripper_index_joint") == 0){
        right_gripper_idx[0] = i;
        ROS_INFO("rgi index: %d", i);
      }
      else if(s.data.compare("right_gripper_thumb_joint") == 0){
        right_gripper_idx[1] = i;
        ROS_INFO("rgt index: %d", i);
      }
      // left arm
      else if(s.data.compare("left_arm_out_joint") == 0){
        left_arm_idx[0] = i;
      }
      else if(s.data.compare("left_arm_fwd_joint") == 0){
        left_arm_idx[1] = i;
      }
      else if(s.data.compare("left_arm_rotate_joint") == 0){
        left_arm_idx[2] = i;
      }
      else if(s.data.compare("left_elbow_joint") == 0){
        left_arm_idx[3] = i;
      }
      else if(s.data.compare("left_wrist_joint") == 0){
        left_arm_idx[4] = i;
      }
      // left gripper
      else if(s.data.compare("left_gripper_index_joint") == 0){
        left_gripper_idx[0] = i;
      }
      else if(s.data.compare("left_gripper_thumb_joint") == 0){
        left_gripper_idx[1] = i;
      }     
    }  
    // clear flag  
    firstFlag = 0;
    ROS_INFO("...done");
  }  

  // Copy position data to appropriate arrays using the index arrays
  if(USE_HEAD){
    for (int i = 0; i < HEAD_SIZE; i++){
      headArray.data[i] = joint_msg->position[head_idx[i]];
    }  
  }   
  if(USE_RIGHT_ARM){
    for (int i = 0; i < RIGHT_ARM_SIZE; i++){
      rightArmArray.data[i] = joint_msg->position[right_arm_idx[i]];
    }  
  }
  if(USE_RIGHT_GRIPPER){
    for (int i = 0; i < RIGHT_GRIPPER_SIZE; i++){
      rightGripperArray.data[i] = joint_msg->position[right_gripper_idx[i]];
    }  
  }
  if(USE_LEFT_ARM){
    for (int i = 0; i < LEFT_ARM_SIZE; i++){
      leftArmArray.data[i] = joint_msg->position[left_arm_idx[i]];
    }  
  }
  if(USE_LEFT_GRIPPER){
    for (int i = 0; i < LEFT_GRIPPER_SIZE; i++){
      leftGripperArray.data[i] = joint_msg->position[left_gripper_idx[i]];
    }  
  }  
  
  /* old version before indecies
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
  */  
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

void printHeader(){
  ROS_INFO("\n\n\n");
  ROS_INFO("\033[32m**********************************************");
  ROS_INFO("\033[32m*           ROS Translate Node               *");
  ROS_INFO("\033[32m*                                            *");
  ROS_INFO("\033[32m* Subscribes to /joint_states and publishes  *");
  ROS_INFO("\033[32m* seperate, sorted arrays for enabled limbs. *");
  ROS_INFO("\033[32m*                                            *");
  ROS_INFO("\033[32m**********************************************");
  ROS_INFO("  ");

  ROS_INFO("Initializing translate node");
  ROS_INFO("Set up publisher(s) and subscriber");
  if(USE_HEAD){
    //nh.loginfo("Using Head");
    ROS_INFO("Using Head");
  }
  if(USE_RIGHT_ARM){
    ROS_INFO("Using Right Arm");
  }
  if(USE_RIGHT_GRIPPER){
    ROS_INFO("Using Right Gripper");
  }
  if(USE_LEFT_ARM){
    ROS_INFO("Using Left Arm");
  }
  if(USE_LEFT_GRIPPER){
    ROS_INFO("Using Left Gripper");
  }     
}


