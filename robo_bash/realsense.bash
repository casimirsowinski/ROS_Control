#!/bin/bash

# Colors
GREEN='\033[0;32m'	# Green
LBLUE='\033[1;34m' 	# Light blue
NC='\033[0m' 		# No color

# Message
printf "\n\n\n\n\t${LBLUE}________________${LBLUE}${GREEN}Run RViz with ... config file${GREEN}${LBLUE}________________${LBLUE}${NC}\n\n\n\n\n"

# Command
roslaunch realsense realsense_r200_launch.launch
wait ${!}
