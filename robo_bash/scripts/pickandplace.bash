#!/bin/bash
# Casimir Sowinski, 2016
# This bash script runs launch files with terminal window control

# Colors
GREEN='\033[0;32m'	# Green
LBLUE='\033[1;34m' 	# Light blue
NC='\033[0m' 		# No color

# Message
printf "\n\n\n\n\t${LBLUE}________________${LBLUE}${GREEN}Loading pick and place python script${GREEN}${LBLUE}________________${LBLUE}${NC}\n\n\n\n\n"

# Command
rosrun robo_arm_nav pick_and_place_demo.py
