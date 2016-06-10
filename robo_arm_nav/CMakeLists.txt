cmake_minimum_required(VERSION 2.8.3)

# based on RBX2 arm nav package
project(robo_arm_nav)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	actionlib
	shape_msgs
	std_srvs
	trajectory_msgs
	sensor_msgs
	diagnostic_msgs
	moveit_msgs
	control_msgs
	tf
	geometry_msgs
	manipulation_msgs
	)

catkin_package()

catkin_python_setup()
