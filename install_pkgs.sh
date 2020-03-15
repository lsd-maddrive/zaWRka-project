#!/bin/bash

CALL_DIR=`dirname $0`
if [ "$CALL_DIR" != "." ]; then
	echo "Must be called inside repository"
	exit 1
fi

if [ -z "$ROS_DISTRO" ]
then
	echo "ROS_DISTRO is not set!"
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
					ros-$ROS_DISTRO-gmapping \
					ros-$ROS_DISTRO-camera-info-manager \
					ros-$ROS_DISTRO-roslint \
					ros-$ROS_DISTRO-image-view \
					ros-$ROS_DISTRO-uvc-camera \
					ros-$ROS_DISTRO-pointcloud-to-laserscan \
					ros-$ROS_DISTRO-usb-cam \
					ros-$ROS_DISTRO-stereo-image-proc \
					libsuitesparse-dev

sudo apt remove ros-$ROS_DISTRO-key-teleop

git -C teleop_tools pull  						|| git clone https://github.com/KaiL4eK/teleop_tools.git
git -C madproto pull  							|| git clone https://github.com/KaiL4eK/madproto.git
git -C wr8_gui_server/smart_vehicle_gui pull 	|| git -C wr8_gui_server clone https://github.com/lilSpeedwagon/smart_vehicle_gui.git
git -C elp_stereo_camera pull					|| git clone https://github.com/KaiL4eK/elp_stereo_camera.git
git -C mad_detector pull						|| git clone https://github.com/KaiL4eK/mad_detector.git

# Install models in local folder
cd mad_detector && ./get_models.sh
