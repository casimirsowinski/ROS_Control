/*
 * Casimir Sowinski, 2016
 * This node subscribes to /joint_states (sensor_msgs::JointState) and publishes 
 * a Float64MultiArray message for the Arduino node over rosserial
 *
 */

// Dependencies
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

// Function prototypes
void joint_cb(const sensor_msgs::JointState::ConstPtr&);

// Global vars
std_msgs::Float64MultiArray multiArray;
const int ARRAY_SIZE = 2;

// Main
int main(int argc, char **argv){
  
  // Pass argc and argv to ros::init() so it can remap CL arguments
  ros::init(argc, argv, "translate");
  
  // Init node so ROS can interact with it
  ros::NodeHandle nh;

  // Subscribe to joint_states to pull all angles, velocities, accelerations, etc.
  ros::Subscriber arduino_joint_sub = nh.subscribe("joint_states", 1000, joint_cb);

  // Publish
  ros::Publisher arduino_joint_pub = nh.advertise<std_msgs::Float64MultiArray>("arduino_joint", 1000);
  

  // Init vars
  ros::Rate loop_rate(10);        // Makes the while loop spin at 20 Hz
  int count = 0;                  // Counts loops    
  
  //std_msgs::Float64MultiArray multiArray;
  multiArray.data.clear();  

  
  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    //assign array a random number between 0 and 255.
    multiArray.data.push_back(0);
  }
  multiArray.data[1] = 222.222;

  
  //broke\\std_msgs::Float64[10] test;  
  //broke\\std_msgs::Float64[10] floatArray = {0};       

  // Loop and handle SIGINT (etc.) interruption
  while (ros::ok()){
    //ROS_INFO("console test message");
    
    // Publish processed array
    arduino_joint_pub.publish(multiArray);

    // spin/wait
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }
 
  return 0;
}

// Function definitions
void joint_cb(const sensor_msgs::JointState::ConstPtr& joint_msg){
  
  ROS_INFO("entered joint_cb");
  
  
  // Copy position data from joint_msg to multiArray
  int i = 0;
  for (int i = 0; i < ARRAY_SIZE; i++){
    multiArray.data[i] = joint_msg->position[i + 3];
  }
  
  /*
  int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = joint_msg->position.begin(); it != joint_msg->position.end(); ++it)
	{
		multiArray.data[i] = *it;
		i++;
	}
	*/
  
  
}





















