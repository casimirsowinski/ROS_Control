#!/bin/bash
###################################################################
# A BASH script to bring up all launch files, python scripts, ect. 
# for the robo ROS project
# Casimir Sowinski 2016
###################################################################

tab="--tab"
cmd="bash -c '~/catkin_ws/src/robo_bash/arbotix.bash';bash"
foo=""

for i in 1 2; do
      foo+=($tab -e "$cmd")         
done

gnome-terminal "${foo[@]}"

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
