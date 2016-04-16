#!/bin/bash
###################################################################
# A BASH script to bring up all launch files, python scripts, ect. 
# for the robo ROS project
# Casimir Sowinski 2016
###################################################################

tab="--tab"
cmd1="bash -c '~/catkin_ws/src/robo_hand_01/robo_bash/arbotix.bash';bash"
cmd2="bash -c '~/catkin_ws/src/robo_hand_01/robo_bash/move_group.bash';bash"
cmd3="bash -c '~/catkin_ws/src/robo_hand_01/robo_bash/rviz.bash';bash"
cmd4="bash -c '~/catkin_ws/src/robo_hand_01/robo_bash/pickandplace.bash';bash"
cmd5="bash -c '~/catkin_ws/src/robo_hand_01/robo_bash/realsense.bash';bash"

delay="sleep 5"

foo=""

foo+=($delay $tab -e "$cmd1")     
foo+=($tab -e "$cmd2")  
foo+=($tab -e "$cmd3") 
#foo+=($tab -e "$cmd4") 
#foo+=($tab -e "$cmd5") 

gnome-terminal "${foo[@]}"

#run1=($tab -e "$cmd1")     
#run2=($tab -e "$cmd2")  
#run3=($tab -e "$cmd3") 
#run4=($tab -e "$cmd4") 

#gnome-terminal "${run1[@]}"
#sleep 1
#gnome-terminal "${run2[@]}"
#sleep 1
#gnome-terminal "${run3[@]}"
#sleep 1
#gnome-terminal "${run4[@]}"


exit 0

# Load ArbotiX simulator
#gnome-terminal --tab -e 'bash -c ~/catkin_ws/src/robo_bash/arbotix.bash';bash

# Load move_group.launch
#gnome-terminal --tab -e bash -c '~/catkin_ws/src/robo_bash/move_group.bash';bash


#exit 0

#####OLD VERSION
# Load ArbotiX simulator
#echo 'Loading ArbotiX simulator'
#printf "\t${LBLUE}________________${LBLUE}${GREEN}Loading ArbotiX simulator${GREEN}${LBLUE}________________${LBLUE}${NC}\n"
#roslaunch robo_bringup robo_with_gripper.launch sim:=true

# Load move_group.launch
#echo 'Loading move_group.launch'
#printf "\t${LBLUE}________________${LBLUE}${GREEN}Loading move_group.launch${GREEN}${LBLUE}________________${LBLUE}${NC}\n"
#roslaunch robo_moveit_config move_group.launch

# Run RViz with ... config file
#echo 'Run RViz with ... config file'
#rosrun rviz rviz -d `rospack find robo_description`/urdf.rviz

# Run the pick and place python script
#echo 'Run the pick and place python script'
#rosrun rbx2_arm_nav moveit_pick_and_place_demo.py