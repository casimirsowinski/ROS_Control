#!/usr/bin/env python

"""
Package: robo_arm_nav
Version: 0.0.1
Description: This is a test script that uses the MoveIt! API to have InMoov go through a 
few simple actions
Maintainer: Casimir Sowinski, "casimirsowinski@gmail.com"
License: BSD
Repo: https://github.com/casimirsowinski/robo_hand_01.git
Author: Casimir Sowinski, "casimirsowinski@gmail.com"
Year: 2016  
"""

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItDemo:
  def __init__(self):
    # Init move_group API
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Init the ROS node
    rospy.init_node('moveit_demo', anonymous=True)
    
    GRIPPER_OPEN = [0.05, 0.05]
    GRIPPER_CLOSED = [0.00, 0.00]
    GRIPPER_NEUTRAL = [0.02, 0.02]
    TEST = 0.0
    
    GRIPPER_TEST = 0.05    
    
    # Connect to the right_arm move group
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    
    # Connect to the right_gripper move group
    right_gripper = moveit_commander.MoveGroupCommander('right_gripper')
    
    # Get the name of the end-effector link
    end_effector_link = right_arm.get_end_effector_link()
    
    # Displat the name of the end_effector link
    rospy.loginfo("The end effector link is: " + str(end_effector_link))
    
    # Set a small tolerance on joint angles
    right_arm.set_goal_joint_tolerance(0.001)
    right_gripper.set_goal_tolerance(0.001)
        
    # Start the arm target in "resting" post stored in the SRDF file
    right_arm.set_named_target('resting')
    
    # Plan a trajectory to the goal configuration
    traj = right_arm.plan()
    
    # Execute the planned trajectory
    right_arm.execute(traj)
    
    # Pause for a moment
    rospy.sleep(1)
    
    rospy.loginfo("1")
    
    # Set the gripper target to neutral position using a joint value target
    right_gripper.set_joint_value_target(0.05)
    
    rospy.loginfo("2")    
    
    # Plan and execute the gripper action
    right_gripper.go()
    
    rospy.loginfo("3")    
    
    rospy.sleep(1)

    # Set target joint values for the arm
    joint_positions = [1, 1, 1, 1, 1]
    
    rospy.loginfo("4")    
    
    # Set the arm's goal configuration to joint positions
    right_arm.set_joint_value_target(joint_positions)
    
    rospy.loginfo("5")            
    
    # Plan and execute the motion
    right_arm.go()
    
    rospy.loginfo("6")    
    
    rospy.sleep(1)
    
    # Save this configuration for later
    right_arm.remember_joint_values('saved_config', joint_positions)
    
    #------
    # Cleanly shut down MoveIt
    moveit_commander.roscpp_shutdown()
    
    # Exit the script
    moveit_commander.os._exit(0)
    
if __name__ == "__main__":
  try: 
    MoveItDemo()
  except rospy.ROSInterruptException:
    pass
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
