/********************************************************************************
Package: robo_setup_tf
Version: 0.0.1
Description: This program was going to be used to try tf trees together and is 
no longer being used.
Maintainer: Casimir Sowinski, "casimirsowinski@gmail.com"
License: BSD
Repo: https://github.com/casimirsowinski/robo_hand_01.git
Author: Casimir Sowinski, "casimirsowinski@gmail.com"
Year: 2016
*******************************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_realsense"));
    r.sleep();
  }
}

