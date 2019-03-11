#!/bin/bash

CALL_DIR=`dirname $0`

CATKIN_WS=`catkin locate`
ret=$?
if [ $ret -ne 0 ]; then
	echo "Must be called inside catkin workspace"
	exit 1
fi

if [ "$CALL_DIR" != "." ]; then
	echo "Must be called inside repository"
	exit 1
fi

CATKIN_SRC_DIR="$CATKIN_WS/src"

git -C $CATKIN_SRC_DIR/rosserial pull 			|| git -C $CATKIN_SRC_DIR clone https://github.com/ros-drivers/rosserial.git
git -C $CATKIN_SRC_DIR/ydlidar pull 			|| git -C $CATKIN_SRC_DIR clone https://github.com/EAIBOT/ydlidar.git
git -C $CATKIN_SRC_DIR/teb_local_planner pull 	|| git -C $CATKIN_SRC_DIR clone https://github.com/rst-tu-dortmund/teb_local_planner.git

git -C wr8_gui_server/smart_vehicle_gui pull || git -C wr8_gui_server clone https://github.com/lilSpeedwagon/smart_vehicle_gui.git

sudo apt purge ros-$ROS_DISTRO-rosserial* \
				ros-$ROS_DISTRO-teb-local-planner* \

sudo apt install ros-$ROS_DISTRO-hector-mapping \
					ros-$ROS_DISTRO-move-base \
					ros-$ROS_DISTRO-gmapping \
					ros-$ROS_DISTRO-global-planner \
					ros-$ROS_DISTRO-costmap-converter \
					ros-$ROS_DISTRO-gazebo9-plugins \
					ros-$ROS_DISTRO-map-server \
					ros-$ROS_DISTRO-amcl \
					ros-$ROS_DISTRO-laser-scan-matcher \
					ros-$ROS_DISTRO-stereo-image-proc \
					ros-$ROS_DISTRO-image-view \
					ros-$ROS_DISTRO-teleop-twist-keyboard \
					ros-$ROS_DISTRO-interactive-markers \
					ros-$ROS_DISTRO-usb-cam \
					ros-$ROS_DISTRO-libg2o \
					ros-$ROS_DISTRO-qt-build

# pip install pygame
cd wr8_software/scripts/graph_path; pyrcc5 -o resources.py my.qrc
