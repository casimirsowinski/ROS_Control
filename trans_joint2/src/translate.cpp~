/*
 * Casimir Sowinski, 2016
 * This node subscribes to /joint_states (sensor_msgs::JointState) and publishes 
 * a Float64MultiArray message for the Arduino node over rosserial
 *
 */

// Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>

// Function prototypes
void joint_cb(const sensor_msgs::JointState::ConstPtr&);

// Main
int main(int argc, char **argv){
  
  // Pass argc and argv to ros::init() so it can remap arguments that were 
  // given in the command line. 
  ros::init(argc, argv, "translate");

  // Init node so ROS can interact with it
  ros::NodeHandle nh;

  // Advertise node to tell the master that it exists, the message type, and
  // the message queue size
  ros::Publisher arduino_joint_pub = nh.advertise<std_msgs::Float64MultiArray>("arduino_joint", 1000);

  ros::Subscriber arduino_joint_sub = nh.subscribe("joint_states", 1000, joint_cb);

  // *****adjust this
  ros::Rate loop_rate(20);

  int count = 0;
  
  std_msgs::Float64MultiArray array;

  // Handle SIGINT (etc.) interruption
  while (ros::ok()){
    

    ROS_INFO("console test message");
    

    arduino_joint_pub.publish(array);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }
 
  return 0;
}

// Function definitions
void joint_cb(const sensor_msgs::JointState::ConstPtr& joint_msg){
  
  ROS_INFO("entered joint_cb");
  
  
  
}





















