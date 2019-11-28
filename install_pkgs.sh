#!/bin/bash

CALL_DIR=`dirname $0`
if [ "$CALL_DIR" != "." ]; then
	echo "Must be called inside repository"
	exit 1
fi

sudo apt install ros-$ROS_DISTRO-base-local-planner \
					ros-$ROS_DISTRO-gazebo-ros-control \
					ros-$ROS_DISTRO-costmap-converter \
					ros-$ROS_DISTRO-libg2o \
					ros-$ROS_DISTRO-move-base \
					ros-$ROS_DISTRO-global-planner \
					ros-$ROS_DISTRO-map-server \
					ros-$ROS_DISTRO-amcl \
					ros-$ROS_DISTRO-controller-interface \
					ros-$ROS_DISTRO-realtime-tools \
					ros-$ROS_DISTRO-urdf \
					ros-$ROS_DISTRO-tf-conversions \
					ros-$ROS_DISTRO-mbf-costmap-core \
					ros-$ROS_DISTRO-mbf-msgs \
					ros-$ROS_DISTRO-tf2-eigen \
					ros-$ROS_DISTRO-compressed-image-transport \
					ros-$ROS_DISTRO-interactive-markers \
					ros-$ROS_DISTRO-key-teleop \
					ros-$ROS_DISTRO-gmapping \
					ros-$ROS_DISTRO-camera-info-manager \
					ros-$ROS_DISTRO-roslint \
					libsuitesparse-dev 


git -C madproto pull  || git clone https://github.com/KaiL4eK/madproto.git
git -C wr8_gui_server/smart_vehicle_gui pull 	|| git -C wr8_gui_server clone https://github.com/lilSpeedwagon/smart_vehicle_gui.git
git -C elp_stereo_camera pull					|| git clone https://github.com/KaiL4eK/elp_stereo_camera.git
