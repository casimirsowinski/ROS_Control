#!/usr/bin/env python

"""
    trajectory_demo.py - Version 0.1 2014-01-14
    
    Send a trajectory to the FollowJointTrajectoryAction server
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

# Modified for Intel capstone program by Casimir Sowinski, 2016

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)
        
        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)
        
        # Arm joints
        right_arm_joints = ['right_arm_out_joint',
                            'right_arm_fwd_joint',
                            'right_arm_rotate_joint',
                            'right_elbow_joint',
                            'right_wrist_joint']
                            
        left_arm_joints = ['left_arm_out_joint',
                            'left_arm_fwd_joint',
                            'left_arm_rotate_joint',
                            'left_elbow_joint',
                            'left_wrist_joint']  
        
        # Which joints define the head?
        ##head_joints = ['head_pan_joint', 'head_tilt_joint']
        head_joints = ['torso_head_tele_joint', 'head_pan_joint', 'head_tilt_joint']
        
        if reset:
            # Set the right arm back to the resting position
            right_arm_goal  = [0, 0, 0, 0, 0, 0]
            
            # Set the left arm back to the resting position            
            left_arm_goal  = [0, 0, 0, 0, 0, 0]
            
            # Re-center the head
            head_goal = [0, 0, 0] 
        else:
            # Set a goal configuration for the right arm
            right_arm_goal  = [0.1, 0.2, -0.2, 0.9, 0.9]
        
            # Set a goal configuration for the left arm
            left_arm_goal  = [0.1, 0.2, -0.2, 0.9, 0.9]
                        
            # Set a goal configuration for the head
            head_goal = [0.05, 0, -0.6]
    
        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')
        
        right_arm_client = actionlib.SimpleActionClient('right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        right_arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
        
        # Connect to the left arm trajectory action server
        rospy.loginfo('Waiting for left arm trajectory controller...')
        
        left_arm_client = actionlib.SimpleActionClient('left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        left_arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
        
        # Connect to the head trajectory action server
        rospy.loginfo('Waiting for head trajectory controller...')
    
        head_client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       
        head_client.wait_for_server()
        
        rospy.loginfo('...connected.')    
    
        # Create a single-point right arm trajectory with the right_arm_goal as the end-point
        right_arm_trajectory = JointTrajectory()
        right_arm_trajectory.joint_names = right_arm_joints
        right_arm_trajectory.points.append(JointTrajectoryPoint())
        right_arm_trajectory.points[0].positions = right_arm_goal
        right_arm_trajectory.points[0].velocities = [0.0 for i in right_arm_joints]
        right_arm_trajectory.points[0].accelerations = [0.0 for i in right_arm_joints]
        right_arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the right arm action server
        rospy.loginfo('Moving the right arm to goal position...')
        
        # Create an empty trajectory goal
        right_arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        right_arm_goal.trajectory = right_arm_trajectory
        
        # Specify zero tolerance for the execution time
        right_arm_goal.goal_time_tolerance = rospy.Duration(0.5)
    
        # Send the goal to the action server
        right_arm_client.send_goal(right_arm_goal)
        
        if not sync:
            # Wait for up to 5 seconds for the motion to complete 
            right_arm_client.wait_for_result(rospy.Duration(5.0))
            
        # Create a single-point left arm trajectory with the left_arm_goal as the end-point
        left_arm_trajectory = JointTrajectory()
        left_arm_trajectory.joint_names = left_arm_joints
        left_arm_trajectory.points.append(JointTrajectoryPoint())
        left_arm_trajectory.points[0].positions = left_arm_goal
        left_arm_trajectory.points[0].velocities = [0.0 for i in left_arm_joints]
        left_arm_trajectory.points[0].accelerations = [0.0 for i in left_arm_joints]
        left_arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the l eft arm action server
        rospy.loginfo('Moving the left arm to goal position...')
        
        # Create an empty trajectory goal
        left_arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        left_arm_goal.trajectory = left_arm_trajectory
        
        # Specify zero tolerance for the execution time
        left_arm_goal.goal_time_tolerance = rospy.Duration(0.5)
    
        # Send the goal to the action server
        left_arm_client.send_goal(left_arm_goal)
        
        if not sync:
            # Wait for up to 5 seconds for the motion to complete 
            left_arm_client.wait_for_result(rospy.Duration(5.0))
        
        # Create a single-point head trajectory with the head_goal as the end-point
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = head_joints
        head_trajectory.points.append(JointTrajectoryPoint())
        head_trajectory.points[0].positions = head_goal
        head_trajectory.points[0].velocities = [0.0 for i in head_joints]
        head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
        head_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the head action server
        rospy.loginfo('Moving the head to goal position...')
        
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = head_trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.5)
    
        # Send the goal
        head_client.send_goal(head_goal)
        
        # Wait for up to 5 seconds for the motion to complete 
        head_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
