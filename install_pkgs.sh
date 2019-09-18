#!/bin/bash

CALL_DIR=`dirname $0`
if [ "$CALL_DIR" != "." ]; then
	echo "Must be called inside repository"
	exit 1
fi

sudo apt install ros-$ROS_DISTRO-base-local-planner \
					ros-$ROS_DISTRO-costmap-converter \
					ros-$ROS_DISTRO-libg2o \
					ros-$ROS_DISTRO-move-base \
					ros-$ROS_DISTRO-global-planner \
					ros-$ROS_DISTRO-map-server \
					ros-$ROS_DISTRO-amcl \
					ros-$ROS_DISTRO-controller-interface \
					libsuitesparse-dev 

# git -C wr8_gui_server/smart_vehicle_gui pull 	|| git -C wr8_gui_server clone https://github.com/lilSpeedwagon/smart_vehicle_gui.git
git -C ackermann_controller pull 		|| git clone https://github.com/KaiL4eK/ackermann_controller.git -b $ROS_DISTRO
