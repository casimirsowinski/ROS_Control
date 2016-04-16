#!/bin/bash

# Colors
GREEN='\033[0;32m'	# Green
LBLUE='\033[1;34m' 	# Light blue
NC='\033[0m' 		# No color

# Message
printf "\n\n\n\n\t${LBLUE}________________${LBLUE}${GREEN}Loading move_group.launch${GREEN}${LBLUE}________________${LBLUE}${NC}\n\n\n\n\n"

# Command
roslaunch robo_moveit_config move_group.launch
wait ${!}
